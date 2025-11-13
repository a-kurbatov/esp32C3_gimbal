#include "imu_msp.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "imu_msp";

// MSP v1 framing
// Host request: '$','M','<', size, cmd, [payload], chksum (XOR of size, cmd, payload)
// FC reply:     '$','M','>', size, cmd, [payload], chksum
// MSP_ATTITUDE payload: int16 roll, int16 pitch, int16 yaw (0.1 deg units)
#define MSP_V1_HDR1 '$'
#define MSP_V1_HDR2 'M'
#define MSP_V1_DIR_TOFC '<'
#define MSP_V1_DIR_FROMFC '>'
#define MSP_ATTITUDE 108

static TaskHandle_t s_task = NULL;
static int s_uart = -1;
static volatile float s_roll_deg = 0.0f;
static volatile float s_pitch_deg = 0.0f;
static volatile float s_yaw_deg = 0.0f;
static volatile uint32_t s_last_us = 0;

static uint8_t msp_checksum_v1(const uint8_t *buf, int len)
{
    uint8_t cs = 0;
    for (int i = 0; i < len; ++i) cs ^= buf[i];
    return cs;
}

static esp_err_t msp_send_req(uint8_t cmd)
{
    uint8_t pkt[6];
    pkt[0] = MSP_V1_HDR1;
    pkt[1] = MSP_V1_HDR2;
    pkt[2] = MSP_V1_DIR_TOFC;
    pkt[3] = 0;        // size
    pkt[4] = cmd;      // cmd
    pkt[5] = msp_checksum_v1(&pkt[3], 2); // XOR(size, cmd)
    int n = uart_write_bytes(s_uart, (const char*)pkt, 6);
    return (n == 6) ? ESP_OK : ESP_FAIL;
}

static int uart_read_exact(uint8_t *dst, int len, int timeout_ms)
{
    int got = 0;
    int t_rem = timeout_ms;
    while (got < len && t_rem >= 0) {
        int r = uart_read_bytes(s_uart, dst + got, len - got, pdMS_TO_TICKS(10));
        if (r > 0) {
            got += r;
        }
        t_rem -= 10;
    }
    return got;
}

static bool msp_read_frame(uint8_t expected_cmd, uint8_t *payload, int *out_size, int timeout_ms)
{
    uint8_t b;
    int t_rem = timeout_ms;
    // Find start '$'
    while (t_rem >= 0) {
        int r = uart_read_bytes(s_uart, &b, 1, pdMS_TO_TICKS(5));
        if (r == 1) {
            if (b == MSP_V1_HDR1) break;
        }
        t_rem -= 5;
    }
    if (t_rem < 0) return false;
    // Expect 'M'
    if (uart_read_exact(&b, 1, t_rem) != 1 || b != MSP_V1_HDR2) return false;
    // Expect '>'
    if (uart_read_exact(&b, 1, t_rem) != 1 || b != MSP_V1_DIR_FROMFC) return false;
    // size, cmd
    uint8_t hdr[2];
    if (uart_read_exact(hdr, 2, t_rem) != 2) return false;
    uint8_t size = hdr[0];
    uint8_t cmd = hdr[1];
    if (cmd != expected_cmd) {
        // Skip payload + checksum
        uint8_t dump[64];
        int to_skip = size + 1;
        while (to_skip > 0) {
            int chunk = to_skip > (int)sizeof(dump) ? (int)sizeof(dump) : to_skip;
            int r = uart_read_exact(dump, chunk, t_rem);
            if (r <= 0) break;
            to_skip -= r;
        }
        return false;
    }
    if (size > 32) return false; // sanity
    if (uart_read_exact(payload, size, t_rem) != size) return false;
    uint8_t cs_recv;
    if (uart_read_exact(&cs_recv, 1, t_rem) != 1) return false;
    uint8_t cs = msp_checksum_v1(hdr, 2);
    cs ^= msp_checksum_v1(payload, size);
    if (cs != cs_recv) return false;
    if (out_size) *out_size = size;
    return true;
}

static void msp_task(void *arg)
{
    const int poll_hz = 50; // adjustable later via Kconfig if needed
    const int poll_ms = 1000 / poll_hz;
    uint8_t payload[16];
    while (1) {
        (void)msp_send_req(MSP_ATTITUDE);
        int size = 0;
        if (msp_read_frame(MSP_ATTITUDE, payload, &size, 50)) {
            if (size >= 6) {
                int16_t roll = (int16_t)(payload[0] | (payload[1] << 8));
                int16_t pitch = (int16_t)(payload[2] | (payload[3] << 8));
                int16_t yaw = (int16_t)(payload[4] | (payload[5] << 8));
                s_roll_deg = roll / 10.0f;
                s_pitch_deg = pitch / 10.0f;
                s_yaw_deg = yaw / 10.0f;
                s_last_us = (uint32_t)esp_timer_get_time();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }
}

esp_err_t imu_msp_init(void)
{
#if CONFIG_GIMBAL_IMU_SELECT_MSP_2AXIS
    s_uart = CONFIG_GIMBAL_MSP_UART_PORT;
    uart_config_t ucfg = {
        .baud_rate = CONFIG_GIMBAL_MSP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(s_uart, 2048, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_uart, &ucfg));
    int tx_io = (CONFIG_GIMBAL_MSP_UART_TX_GPIO < 0) ? UART_PIN_NO_CHANGE : CONFIG_GIMBAL_MSP_UART_TX_GPIO;
    ESP_ERROR_CHECK(uart_set_pin(s_uart, tx_io, CONFIG_GIMBAL_MSP_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(msp_task, "msp_poll", 3072, NULL, 4, &s_task);
    ESP_LOGI(TAG, "MSP UART%d RX=%d TX=%d baud=%d", s_uart, CONFIG_GIMBAL_MSP_UART_RX_GPIO, CONFIG_GIMBAL_MSP_UART_TX_GPIO, CONFIG_GIMBAL_MSP_UART_BAUD);
    return ESP_OK;
#else
    ESP_LOGW(TAG, "MSP 2-axis mode not selected; imu_msp_init() is a no-op");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t imu_msp_read(msp_att_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint32_t now = (uint32_t)esp_timer_get_time();
    out->roll_deg = s_roll_deg;
    out->pitch_deg = s_pitch_deg;
    out->yaw_deg = s_yaw_deg;
    out->age_ms = s_last_us ? (now - s_last_us) / 1000 : 0xFFFFFFFF;
    return (s_last_us != 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

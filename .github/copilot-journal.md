2025-11-19

Context:
- User reported noisy servo behaviour from the PID-controlled 1-axis gimbal. Working from `main.c` in the ESP-IDF project.

Decisions:
- Improve PID by adding a small deadband, integral anti-windup (clamped integral term), and a low-pass filter on the derivative term. Keep changes local to `main.c` and avoid touching other components.

Changes made:
- `main.c`: Extended `pid_t` with `d_filtered` and `i_max`.
- `pid_init()` now accepts an `i_max` parameter used to clamp the integral term.
- `pid_update()` now implements:
  - deadband (0.15°) to avoid tiny continuous corrections,
  - clamped integral term to prevent windup,
  - filtered derivative (simple exponential LPF with alpha=0.85),
  - output clamping to the antenna limits.
- Updated `app_main` to pass `CONFIG_GIMBAL_ANTENNA_LIMIT_DEG` as `i_max` and to pass `ant_lim` to `pid_update()`.

Open questions / risks:
- The deadband value and derivative filter alpha are tuned conservatively; you may want to adjust them after observing behaviour on the real system.
- Integral-clamp uses `CONFIG_GIMBAL_ANTENNA_LIMIT_DEG` as a simple bound; more precise tuning of `i_max` relative to `ki` may reduce steady-state error.
- These changes should reduce jitter but may increase steady-state offset if `ki` is low/zero (current default `ki=0`).

Next actions:
- Flash and test on hardware. Observe step/response and tune `CONFIG_GIMBAL_PID_*` and the deadband/alpha constants.
- If further suppression is needed, consider:
  - adding a small motion deadband before commanding the servo (i.e., suppress small servo pulses),
  - implementing derivative-on-measurement instead of derivative-on-error, or
  - smoothing the IMU-derived pitch further.
# Copilot Project Journal (append-only)

Purpose: Track decisions, changes, and open items. Append a new dated entry every iteration. Never rewrite past entries.

Entry template
* Date: YYYY-MM-DD
* Context: What changed this iteration (files, settings, hardware assumptions)
* Decisions: Finalized choices (pins, APIs, parameters)
* Changes made: Commits/edits and their intent
* Open questions / risks
* Next actions for the agent

## Entries

Date: 2025-11-03
Context:
* Initialized ESP-IDF project for ESP32-C3 (Super Mini), targeting IDF 5.5.1.
* Current IMU focus: ICM-42688-P (SPI by default); keep BNO08x adapter available for easy swap.
* Created components: servo_pwm (LEDC 50 Hz), button (long-press), imu adapters (factory + ICM-42688-P SPI stub + BNO08x I2C stub), main app (NVS offset, complementary filter + PID at 10 Hz).
Decisions:
* Pins (ESP32-C3): SPI SCLK=GPIO4, MOSI=GPIO6, MISO=GPIO5, CS=GPIO7; I2C SDA=GPIO8, SCL=GPIO9 (reserved); Servo PWM=GPIO10; Button=GPIO1.
* Mechanics: ±32° antenna, ±105° servo, ratio 3.28. Servo pulse 500–2500 µs.
* Calibration: long-press 2 s, store offset in NVS.
Changes made:
* Added CMakeLists.txt, sdkconfig.defaults, Kconfig for project options.
* Implemented servo_pwm, button, imu factory, ICM-42688-P SPI WHO_AM_I stub, BNO08x I2C stub, and main control loop.
* Updated copilot-instructions with pin maps, power, and build notes.
Open questions / risks:
* Replace ICM-42688-P stub with full driver and confirm axis mapping for pitch.
* DC-DC module choice for 10–50 V input; verify servo current headroom and common ground.
* Confirm COM port auto-detect behavior on target PC (COM10 typical).
Next actions:
* Integrate a production ICM-42688-P driver (SPI), validate WHO_AM_I and streaming reads.
* Add orientation config and tuning for PID gains; add logging of raw accel/gyro.
* Provide README with wiring and flashing steps.

Date: 2025-11-03
Context:
* Build failed due to Kconfig "float" type (not supported by IDF kconfig) and new driver header paths in IDF 5.5.1.
Decisions:
* Replace float Kconfig options (alpha, PID gains, mech ratio) with int scaled by 1000.
* Use new esp_driver_* headers and declare component deps accordingly.
Changes made:
* Updated components/gimbal_config/Kconfig.projbuild to use int x1000 for alpha, kp, ki, kd, ratio; updated sdkconfig.defaults and main.c to convert to float.
* Switched includes to esp_driver_gpio/ledc/i2c/spi and updated component CMake REQUIRES.
* Added gimbal_config header and CMake to provide fallbacks for CONFIG_*.
Open questions / risks:
* Still using ICM‑42688‑P SPI stub; real driver integration pending.
Next actions:
* Rebuild after clearing sdkconfig; validate no further compile errors; then implement full ICM‑42688‑P reads.
* Consider integrating attached "ESP32-ICM426XX-driver" as an optional component behind the IMU adapter (likely I2C path via I2Cdev).

Date: 2025-11-03
Context:
* Link failed: undefined references to imu_icm42688p_* from imu_factory.
Decision:
* Make the imu factory component depend on the specific IMU driver components to enforce link order in IDF.
Changes made:
* Updated components/imu/CMakeLists.txt to REQUIRES imu_icm42688p imu_bno08x.
Expected result:
* Linker resolves symbols from driver libraries.

Date: 2025-11-04
Context:
* Build succeeded on ESP-IDF 5.5.1 (ESP32‑C3). One benign warning remains in `imu_factory.c` (`not_impl` unused).
Decisions:
* Keep the warning for now (cosmetic). Option to silence later with conditional compile or unused attribute.
* Servo PWM uses LEDC at 50 Hz with 14‑bit resolution on C3; button component depends on `esp_timer`.
Changes made (since previous entry):
* Fixed LEDC resolution (14‑bit), split clamp lines, added `esp_timer` dependency.
* Restored correct driver header paths (`driver/*`) and ensured component deps are set.
* Link order fixed by making `imu` component require the specific IMU drivers.
Open questions / risks:
* IMU data is still stubbed; need real ICM‑42688‑P reads and axis mapping for pitch.
* Decide whether to integrate the attached ICM426XX I2C driver now or continue with SPI.
Next actions:
* Integrate ICM‑42688‑P real driver (prefer I2C ICM426XX now for speed) behind the adapter.
* Add a simple README with wiring, flashing, and default pins.
* Add Kconfig guard so BNO08x stub isn't compiled unless selected (compile speed).

IDF environment + flashing (Windows PowerShell)
* If 'idf.py' is not recognized in PowerShell, run via the batch export (avoids execution policy issues):
	- cmd /c "C:\\Espressif\\frameworks\\esp-idf-v5.5.1\\export.bat && idf.py -p COM10 flash monitor"
* To auto-detect the serial port, drop the -p option: idf.py flash monitor
* If monitor doesn't open, check the COM port in Device Manager or press BOOT (if present) while resetting to enter download mode.

Date: 2025-11-05
Context:
* No IMU connected yet; need to run without hardware.
Decisions:
* Added IMU emulator mode as a third selectable option in Kconfig; set as default in sdkconfig.defaults.
* Emulator generates a 15 s cycle: +30° pitch, +30° yaw, +30° roll, -30° yaw, -30° pitch, -30° roll.
* Logs downsampled to ~2 Hz and now include raw IMU values and last servo PWM pulse width.
Changes made:
* New component `components/imu_emulator` with `imu_emulator_init/read`.
* Updated `imu_factory` to support emulator, Kconfig to add EMULATOR option, and defaults switched to emulator.
* Added `servo_pwm_get_last_us()` to report actual PWM pulse; updated main to log IMU raw values + PWM at 2 Hz.
 * Reverted SPI fallback (mode 3 retry) in `imu_icm42688p.c` per user note; current SPI driver remains simple mode-0.
Next actions:
* After wiring the IMU, switch Kconfig to real driver and verify WHO_AM_I.
* Optionally integrate the ICM426XX I2C driver for faster bring-up.

When emulator is no longer needed (cleanup checklist)
1) Switch to real IMU in config
	- Run `idf.py menuconfig` → Gimbal Project → IMU driver → select `ICM-42688-P` (or `BNO08x`). Save and rebuild.
	- Commit `sdkconfig` or update `sdkconfig.defaults` to set `CONFIG_GIMBAL_IMU_SELECT_ICM42688P=y` and remove `CONFIG_GIMBAL_IMU_SELECT_EMULATOR`.
2) Optional: remove emulator code from the build to shrink binary
	- Delete folder `components/imu_emulator/` (or keep it if you want the option available).
	- Edit `components/imu/CMakeLists.txt` and remove `imu_emulator` from the `REQUIRES` list (it’s harmless to leave, but this reduces link time/size).
	- In `components/gimbal_config/Kconfig.projbuild`, you may set the default of the IMU choice back to `ICM-42688-P`.
3) Keep or adjust logging
	- The 2 Hz status logs can remain enabled; for production, lower the verbosity or guard with a Kconfig option.
4) For SPI bring-up (ICM-42688-P)
	- If WHO_AM_I is not 0x47, first check wiring and VDDIO. If still failing, you may temporarily lower SPI speed to 400 kHz and add a 5 ms power-up delay. Mode‑3 retry code was intentionally reverted; only re‑enable if a specific breakout requires it.

	Date: 2025-11-07
	Context:
	* Requested SPI GPIO remap.
	Decision:
	* Remapped SPI pins in `sdkconfig.defaults`:
		- SCLK=GPIO5, MISO=GPIO6, MOSI=GPIO7, CS=GPIO8.
	Impact/notes:
	* If you previously built, `sdkconfig` may still hold old values; run `idf.py menuconfig` and set pins under Gimbal Project, or delete `sdkconfig` to pick up defaults.
	* GPIO8 is no longer free for I2C SDA; if you later switch to I2C, choose other SDA/SCL pins or revert CS to GPIO7.

	Build note (2025-11-05):
	* Added `esp_timer` as a private requirement of the `imu_emulator` component to resolve `esp_timer.h` include during compilation.

	How to enable the IMU emulator (if logs still show icm42688p)
	1) The active configuration is in `sdkconfig` (not `sdkconfig.defaults`). If you previously built with ICM-42688-P, it remains selected.
	2) In a terminal with ESP-IDF exported, run: `idf.py menuconfig` → Gimbal Project → IMU driver → select "Emulator (no hardware)" → Save → Exit.
	3) Or delete the `sdkconfig` file and rebuild to pick up `sdkconfig.defaults` (which selects Emulator by default).
	4) Rebuild/flash: `idf.py flash monitor`. Expected: no `icm42688p` logs; 2 Hz log lines will show changing ax/ay/az (pitch/roll segments) and nonzero gz during yaw segments.

Date: 2025-11-07
Context:
* User switched IMU to BNO085 (BNO08x) in menuconfig and requested using real IMU input.
Decisions:
* Updated `sdkconfig.defaults` to select `CONFIG_GIMBAL_IMU_SELECT_BNO08X=y` by default so fresh configs use BNO08x.
* Keep I2C defaults: SDA=GPIO8, SCL=GPIO9, Freq=400 kHz, Addr=0x4A (adjust if your board uses 0x4B).
Changes made:
* Edited `sdkconfig.defaults` to comment out Emulator and enable BNO08x.
* Added a one-time runtime warning in `imu_bno08x_read()` clarifying the driver is a stub and returns placeholder data until SH-2/SHTP support is integrated.
Open questions / risks:
* The current BNO08x component is a stub; real sensor data requires integrating an SH-2/SHTP driver (Hillcrest SH2 + SHTP or SparkFun/Adafruit libs under permissive licenses). Without it, IMU values will remain static (ax=0, ay=0, az=1, gx=gy=gz=0).
* Confirm I2C wiring to GPIO8/9 and the device address (0x4A vs 0x4B). Ensure pull-ups are present (internal pull-ups enabled in code, but external 2.2–4.7 kΩ recommended).
Next actions:
* Choose and integrate a BNO08x driver:
	- Option A: Vendor SH2/SHTP (Hillcrest) C sources (BSD) minimal subset (reports: Rotation Vector or Game Rotation Vector).
	- Option B: Port SparkFun BNO080 library to ESP-IDF C component.
* After driver integration: enable desired reports at init, read reports in `imu_bno08x_read()`, map to `imu_sample_t` (accel/gyro or fused pitch).

Date: 2025-11-07
Context:
* User chose to use BNO08x over SPI and confirmed pinout: SCLK=5, MISO=6, MOSI=7, CS=8, INT=9.
Decisions:
* Add BNO08x transport selection (I2C/SPI) in Kconfig; default to SPI.
* Initialize SPI2 bus and attach BNO08x device at 1 MHz, mode 0. Keep INT=9 configured as input w/ pull-up.
Changes made:
* Kconfig: added `choice GIMBAL_BNO08X_TRANSPORT` with `GIMBAL_BNO08X_USE_SPI` default; added `GIMBAL_BNO08X_INT_GPIO` earlier.
* imu_bno08x: conditional init for I2C vs SPI, added SPI bring-up and logs; added `esp_driver_spi` dependency in CMake.
* sdkconfig.defaults: set `CONFIG_GIMBAL_IMU_SELECT_BNO08X=y` and `CONFIG_GIMBAL_BNO08X_USE_SPI=y`; keep INT=9.
Open questions / risks:
* Driver still a stub. Need SH-2/SHTP over SPI implementation to get real data. Module must be in SPI mode (PS0/PS1 strapped LOW; check your breakout jumpers/solder bridges). WAKE may need handling.
Next actions:
* Integrate SH-2/SHTP over SPI (or allow I2C fallback). Then map rotation/accel/gyro reports into `imu_sample_t`.

Date: 2025-11-07
Context:
* User requested mapping the BNO08x INT pin and confirmed pin plan for SPI lines.
Decisions:
* Map BNO08x INT to GPIO9 via new Kconfig `CONFIG_GIMBAL_BNO08X_INT_GPIO` (defaults to 9 in `sdkconfig.defaults`).
* Keep existing SPI mapping (used previously for ICM): SCLK=GPIO5, MISO=GPIO6, MOSI=GPIO7, CS=GPIO8.
Changes made:
* Added `GIMBAL_BNO08X_INT_GPIO` to Kconfig and fallback to `gimbal_config.h`.
* Configure INT pin as input with pull-up and log mapping in `imu_bno08x_init()`.
Open questions / risks:
* Pin conflict: GPIO9 is I2C SCL by default. If BNO08x is to run over I2C, move INT to another free GPIO (e.g., 2) or leave it unconnected. If BNO08x is to run over SPI (SCLK=5, MISO=6, MOSI=7, CS=8, INT=9), the breakout must be configured for SPI (PS0/PS1 low) and our BNO08x component must be migrated from I2C to SPI (SHTP-over-SPI implementation pending).
Next actions:
* Decide transport for BNO08x:
	- I2C (fastest path): keep SDA=8/SCL=9; use INT on another pin (or none). Integrate SH-2/SHTP over I2C driver.
	- SPI (matches current wiring plan): keep 5/6/7/8 + INT=9; I will add SPI transport and SH-2/SHTP over SPI, update CMake deps to `esp_driver_spi`, and Kconfig to select BNO08x SPI.

Date: 2025-11-07
Context:
* Integrated BNO08x over SPI using Hillcrest SH-2/SHTP stack scaffolding; SPI+INT already verified (SHTP packets seen).
Decisions:
* Vendor SH-2/SHTP (Apache-2.0) minimal subset under `components/sh2/`. Provide an ESP-IDF SPI HAL and wire the IMU task to service SH-2.
* Enable Accelerometer and Calibrated Gyro at 100 Hz initially; return samples to the existing control loop.
Changes made:
* New component `components/sh2/` with headers (`sh2.h`, `shtp.h`, `sh2_err.h`, `sh2_util.{h,c}`, `sh2_SensorValue.{h,c}`) and ESP-IDF SPI HAL (`sh2_hal_espidf_spi.{c,h}`). CMake updated to include ESP timer/SPI deps.
* `imu_bno08x.c`: replaced ad-hoc SPI reads with SH-2 flow: configure HAL with SPI+INT, start a FreeRTOS task that opens SH-2, registers sensor callback, enables accel+gyro, and calls `sh2_service()`.
* Kconfig/Defaults: already set BNO08x SPI path and INT=GPIO9.
* Build fixes: expanded `sh2.h` with missing SH-2 types (error/counts, osc/cal enums, tare/quaternion, FRS_ID_META_*), async event union fields; added SHTP tag defines to `shtp.h`; removed stale `sh2_init()` call.
Open questions / risks:
* Ensure `components/sh2/sh2.c` and `shtp.c` are the full upstream implementations (Apache-2.0). If placeholders remain, replace with vendor sources to avoid runtime issues.
* Verify module is in SPI mode (PS0/PS1 LOW) and WAKE/RST handling is acceptable for continuous operation.
* Heap/stack sizing for SH-2 task (currently 4096 bytes) may need tuning when full stack is used.
Next actions:
* Replace any placeholder `sh2.c`/`shtp.c` with upstream versions; rebuild.
* Confirm logs show SH-2 reset/product ID and feature config success; verify accel/gyro nonzero and control loop responds.
* Optionally enable Rotation Vector and use quaternion-derived tilt; then tune complementary filter and PID gains on hardware.

Date: 2025-11-08
Context:
* SPI mode 3 confirmed; SHTP adverts are visible. IMU RST pin is now available on hardware.
Decisions:
* Map BNO08x RST to ESP32-C3 GPIO2.
* Increase SH-2 servicing density when INT is asserted to quickly drain advert fragments.
Changes made:
* Updated `sdkconfig.defaults`: `CONFIG_GIMBAL_BNO08X_RST_GPIO=2` (HAL will pulse reset on open).
* `imu_bno08x`: in the main SH-2 task loop, when INT=0, call `sh2_service()` in a small burst to consume all fragments; keep a small delay otherwise to avoid WDT.
* Moved I2C default pins to avoid conflicts with SPI and INT: `SDA=4`, `SCL=3` (were `8` and `9`). SPI remains `SCLK=5, MOSI=7, MISO=6, CS=8`; BNO INT=`9`.
* Set default IDF target to ESP32‑C3 and default BNO08x SPI mode to 3 in `sdkconfig.defaults`.
Open questions / risks:
* If WAKE is not present on this board, ensure the part is strapped awake; otherwise rely on SH‑2 `devReset/devOn` to wake.
* If SH‑2 still doesn’t progress after reset, verify BOOT is not held low and PS0/PS1 are strapped for SPI.
Next actions:
* Rebuild/flash. Expect “SH2 reset complete”, then successful `getProdIds`, then feature enable and first ACC/GYR event. If not, share the SPI trace dump of the first header+payload packets.

Date: 2025-11-08
Context:
* Added a UART-RVC sniffer mode to validate BNO08x output independent of SH-2/SHTP transport.
* User re-strapped PS0/PS1 for UART test (PS1=HIGH, PS0=LOW). BNO085 RX remains connected; TX available on module pin labeled GPIO6.
Decisions:
* Introduce Kconfig `CONFIG_GIMBAL_UART_RVC_SNIFFER` to disable IMU/SPI init and start a UART reader instead.
* Default UART: PORT=1, RX=GPIO0 (ESP32-C3), TX disabled (-1), BAUD=115200.
* When sniffer is enabled, set SPI pins (SCLK/MOSI/CS) to high-impedance inputs and leave INT as input (no drive on sensor lines).
Changes made:
* Kconfig: added UART sniffer options (port, RX/TX pins, baud).
* gimbal_config.h: fallback macros for the new options.
* main.c: guarded sniffer path that initializes UART and logs hex dumps of incoming bytes; SPI pins forced Hi-Z; IMU task bypassed.
* sdkconfig.defaults: enabled `CONFIG_GIMBAL_UART_RVC_SNIFFER=y` with defaults (UART1 RX GPIO0).
Open questions / risks:
* Confirm which ESP32-C3 pin is actually wired to the BNO TX (module GPIO6). Defaults assume GPIO0; adjust `CONFIG_GIMBAL_UART_RX_GPIO` if different.
* If TX (from ESP32 to BNO) is physically connected, keeping TX unconfigured (-1) avoids driving it; if needed, map TX to a safe unused pin.
Next actions:
* Build and flash. Expect repeating 0xAA-prefixed frames at ~50–100 Hz in the log. If observed, the sensor is healthy; proceed to re-enable SPI and continue SH-2 debugging.

Date: 2025-11-08
Context:
* User confirmed RVC stream present (0xAA AA frames). Proceeding to restore SPI path and refine activation.
Decisions:
* Disable INT-bypass reads (caused zero-length packets earlier).
* Increase post-reset wait to 300 ms before enabling features.
* Enable Game Rotation Vector at 50 Hz alongside Accelerometer/Gyro to jump-start streaming.
* Add sh2_getCounts() logging to verify features are ON at runtime.
Changes made:
* imu_bno08x.c: delay -> 300 ms; add GRV enable; remove bypass call; add counts in health log.
* sdkconfig.defaults: disable UART sniffer by default; set UART RX GPIO=6 for future tests.
Open questions / risks:
* If no sensor events after these changes, verify INT pull-up and polarity, and consider increasing delay further (500 ms).
Next actions:
* Rebuild/flash with SPI straps (PS0=LOW, PS1=LOW on your board). Expect SH2 reset, setSensorConfig rc=0, then ACC/GYR events. Share logs if still silent.

Date: 2025-11-08
Context:
* Flashing failed due to wrong chip target (esp32 instead of esp32c3) even after deleting sdkconfig.
Decisions:
* Enforce default target in CMake to ESP32-C3 so fresh configs don’t fall back to esp32.
Changes made:
* CMakeLists.txt: set(IDF_TARGET esp32c3 CACHE STRING "IDF Target" FORCE) before including project.cmake.
Open questions / risks:
* Users can still have stale build cache; recommend running `idf.py set-target esp32c3` or cleaning build/ if mismatch persists.
Next actions:
* Reconfigure (set-target) or clean build, then build/flash again.

Date: 2025-11-08
Context:
* UART-RVC verified streaming (0xAA AA frames). Returned to SPI bring-up.
* Implemented SHTP-over-SPI fixes: 4-byte boundary padding on RX/TX with CS held active; longer post-reset wait; optional host reinitialize; enable GRV+ACC+GYR; bounded GET_FEATURE/COUNTS queries.
* Observed two states in logs:
	- With original MOSI/MISO mapping: valid adverts + SH2 reset, setFeature rc=0, but no events and control queries time out (likely MOSI not reaching hub).
	- After swapping MOSI/MISO in menuconfig: first header 0xFFFF → MISO floating (wrong swap).
Decisions:
* Confirm breakout pin semantics for GY-BNO08X: SDA=SDI=MOSI (host→IMU), ADO=SDO=MISO (IMU→host), SCL=SCK.
* Keep ESP32-C3 mapping: SCK=GPIO5 → SCL, MOSI=GPIO7 → SDA, MISO=GPIO6 → ADO, CS=GPIO8, INT=GPIO9, RST=GPIO2.
* Avoid INT-bypass; read only on INT low. Keep SPI at 1 MHz, mode 3.
Changes made:
* sh2_hal_espidf_spi.c: add 4-byte padding on SPI reads/writes with CS keep-active across payload+pad.
* imu_bno08x.c: 500 ms post-reset delay, optional sh2_reinitialize(), enable GRV+ACC+GYR, bounded GET_FEATURE logging, reduced service burst and unconditional yield to prevent WDT.
* sh2.c: timeouts for GET_FEATURE and GET_COUNTS to prevent stalls.
Open questions / risks:
* If wiring still mismatched, adverts or headers will corrupt (0xFFFF). Once wiring is correct, expect GET_FEATURE responses then sensor events.
Next actions:
* Restore MOSI=GPIO7 (SDA), MISO=GPIO6 (ADO). Clean build (delete build/ and sdkconfig), rebuild/flash. Verify adverts, then check GET_FEATURE logs and sensor events.

Date: 2025-11-09
Context:
* User re-wired to alternate ESP32-C3 SPI pins and requested defaults update.
Decisions:
* Adopt new SPI mapping in defaults to match wiring: SCLK=GPIO4, MOSI=GPIO6, MISO=GPIO5, CS=GPIO7; keep INT=GPIO9, RST=GPIO2; SPI mode 3.
Changes made:
* sdkconfig.defaults: updated CONFIG_GIMBAL_SPI_* pins accordingly.
Open questions / risks:
* Ensure physical wiring matches: MOSI→SDA (SDI), MISO←ADO (SDO). Power-cycle after strap changes.
Next actions:
* Delete sdkconfig or run menuconfig to pick up new defaults; build/flash; verify GET_FEATURE responses and sensor events.

Date: 2025-11-09
Context:
* SPI bring-up remained flaky despite valid adverts: setFeature/reinit timed out, indicating writes not being observed by the hub. Root cause: GY‑BNO08X clone silkscreen ambiguity (SDA/ADO labels reversed for SPI on this unit) led to recurring miswires and intermittent CS/MOSI integrity issues. UART‑RVC proved the sensor is healthy.
Decisions:
* Switch BNO08x transport to I2C to simplify bring-up and avoid SPI CS/padding/timing pitfalls. Reuse existing wires where possible.
Changes made:
* Implemented SH‑2/SHTP I2C HAL: `sh2_hal_espidf_i2c.{c,h}` with INT-gated reads, header+payload transfers, and a compatible bypass window API.
* Integrated I2C HAL into `imu_bno08x`: conditional HAL selection, shared service task/callbacks, and logging.
* Updated `components/sh2/CMakeLists.txt` to include I2C HAL and require `esp_driver_i2c`.
* Defaults: set BNO08x transport to I2C; SDA=GPIO6, SCL=GPIO4; INT=GPIO9; RST=GPIO2; I2C 100 kHz; addr 0x4B (AD0/ADDR pulled HIGH on this board).
Reason for switching to I2C:
* The specific breakout’s SDA/ADO labelling for SPI was counterintuitive (SDA pad acted as SDO on this unit), causing repeated MOSI/MISO inversions and unreliable CS/MOSI connectivity. I2C removes CS and SPI padding/mode concerns, needs only SDA/SCL with pull-ups, and is sufficient bandwidth for 100 Hz accel/gyro/GRV.
Open questions / risks:
* Ensure straps set for I2C on the breakout; confirm pull-ups on SDA/SCL. If the board lacks pull-ups, add 2.2–4.7 kΩ to 3.3 V.
Next actions:
* Rebuild with I2C enabled and new pin defaults (delete sdkconfig or use menuconfig). Wire SDA=GPIO6, SCL=GPIO4, INT=GPIO9, RST=GPIO2. Expect adverts → GET_FEATURE logs → ACC/GYR events. If not, capture first 50 log lines. If ready stays 0, consider explicit advertise request or return to SPI with WAKE high.

Date: 2025-11-09
Context:
* I2C ACK achieved at 0x4B but adverts/RESET not observed reliably despite extended bypass/polling; SPI path previously verified with good adverts. Datasheet clarifies PS0 becomes WAKE in SPI.
Decisions:
* Revert to SPI (mode 3) and use WAKE control on ESP32‑C3 GPIO3 to guarantee the hub is awake.
Changes made:
* sdkconfig.defaults: transport set to SPI; SPI pins SCLK=4, MOSI=6, MISO=5, CS=7; INT=9; RST=2; WAKE=3.
* Existing SPI HAL already asserts WAKE high in hal_open(); no further code needed beyond defaults.
Rationale:
* SPI with WAKE high avoids I2C strap ambiguity on this breakout and leverages the already working SHTP-over-SPI path (with 4‑byte padding and timing fixes). This should yield ACC/GYR events quickly.
Next actions:
* Power-cycle with PS0=LOW, PS1=LOW (SPI straps). Connect WAKE (PS0) to GPIO3. Rebuild/flash. Expect: SH2 RESET → setFeature rc=0 → sensor events; health shows acc_on/gyro_on > 0.

| 2025-11-10 | `imu_bno08x.c` | **Hypothesis:** The `sh2_reinitialize()` command is the problem. | Commented out the call to `sh2_reinitialize()` to see if subsequent commands (like `setSensorConfig`) would succeed. | `idf_py_stdout_output_26148` | **Failed.** `setSensorConfig` logs `rc=0` (as it doesn't wait for a reply), but the debug log `dbg: ... rc_a=-6 rc_g=-6` proves all commands are still timing out. |
| 2025-11-10 | `BNO080_085-Datasheet.pdf` | **Datasheet Analysis:** Found critical new information. | The datasheet explicitly states on Page 46 that the BNO08x **requires SPI Mode 3 (CPOL=1, CPHA=1)**. | N/A | Our test of Mode 0 was incorrect. We must use Mode 3. We have not yet tested the *correct* combination of Mode 3 *and* no software resets. |
| 2025-11-10 | `sdkconfig.defaults` | **Action:** Revert to correct SPI Mode 3. | Changed `CONFIG_GIMBAL_BNO08X_SPI_MODE` back to `3` in `sdkconfig.defaults` to match the datasheet. Kept software resets disabled. | `idf_py_stdout_output_20104` | **Failed.** The failure is identical (`rc_a=-6`). This proves the issue is not *just* the SPI mode or *just* the software resets. |
| 2025-11-10 | `BNO080_085-Datasheet.pdf` | **Hypothesis:** The sensor is going to sleep. | Datasheet (Page 19) requires an **active-low WAKE pulse** before *every* command to wake the sensor. Our code only does this once at boot. | N/A | The sensor is likely sleeping after its initial advertisement, causing all subsequent commands to fail. |
| 2025-11-10 | `sh2_hal_espidf_spi.c` | **Action:** Implement WAKE-on-write protocol. | Added a `hal_wake()` function to pulse the WAKE pin **low** for 500us. Modified `hal_write()` to call `hal_wake()` before every SPI write. | `idf_py_stdout_output_14664` | **Failed.** The `dbg:` log still shows `rc_a=-6 rc_g=-6`. The failure mode is unchanged, even with the correct WAKE pulse. |
| 2025-11-10 | `BNO080_085-Datasheet.pdf` | **Hypothesis:** Hardware reset sequence is invalid. | Datasheet (Page 18, Note 5) requires the WAKE (PS0) pin to be **held high** *during* the hardware reset to select SPI mode. Our `hal_open` function was pulsing it low. | N/A | This protocol violation could be putting the sensor in an invalid state. |
| 2025-11-10 | `sh2_hal_espidf_spi.c` | **Action:** Fix hardware reset sequence. | Simplified the `hal_open` function to *only* set the WAKE pin high, then pulse the RESET pin low, exactly as the datasheet requires. | `idf_py_stdout_output_25988` | **Failed.** The log is identical to the previous one. The `dbg:` log still shows `rc_a=-6 rc_g=-6`. |

Context:
* Simplified BNO08x activation assuming IMU is ready after HAL reset.
* Removed task-start `sh2_devReset()` and host `sh2_reinitialize()` gating.
Changes made:
* `components/imu_bno08x/imu_bno08x.c`: after receiving `SH2_RESET`, immediately call `sh2_setSensorConfig()` for ACCELEROMETER and GYROSCOPE_CALIBRATED at 100 Hz; keep servicing SHTP with INT-aware loop.
Observed result (monitor):
* `SH2 reset complete` → `sh2_open rc=0` → two `SHTP tx len=21` (set-feature) → `setSensorConfig accel rc=0 gyro rc=0`.
* No sensor events yet; periodic health: `int=1 ready=1 enabled=1 last_evt_ms=4294967295`.
Decisions / hypotheses:
* Some firmwares don’t emit a reply to Initialize; our new flow avoids waiting for it. However, lack of events suggests either: timing window after reset, missed INT, or hub not actually starting the features.
Next actions:
* Lower SPI clock to 1 MHz and retry.
* Enable optional SPI RX trace in HAL to dump first payload bytes of control responses and any sensor traffic.
* Add a short delay (50–100 ms) between `SH2_RESET` and `setSensorConfig`, and log a single `sh2_getSensorConfig()` readback to verify the feature is active.
* For diagnosis, temporarily bypass INT gating (poll header every 10–20 ms) to rule out missed INT edges on this board.

Date: 2025-11-10
Context:
* Hardware fix confirmed: Removed the PS0 (WAKE) strap/jumper on the BNO08x breakout so WAKE/PS0 is no longer forced by the board. WAKE is now driven only by ESP32‑C3 GPIO3.
* Result: SH-2 init completes and IMU events stream normally; `gimbal_main` shows non-zero accel/gyro and stable updates. Periodic health log no longer reports timeouts (no `rc_a=-6 rc_g=-6`).
Decisions:
* Keep SPI Mode 3 and current pin mapping: SCLK=GPIO4, MOSI=GPIO6, MISO=GPIO5, CS=GPIO7; INT (active‑low)=GPIO9; RST=GPIO2; WAKE=GPIO3.
* Retain WAKE-on-write in the SPI HAL (pulse WAKE low ~500 µs before each command) to adhere to the wake protocol and for robustness.
Changes made:
* No code changes for this fix; hardware-only change (removed PS0 jumper). Verified `setSensorConfig` for ACCEL and GYRO succeeds and `sensor_cb` receives events.
Evidence:
* Monitor: "SH2 reset complete", "setSensorConfig accel rc=0", "gyro rc=0", followed by ACC/GYR event logs and valid IMU values in main output.
Open questions / risks:
* If straps are altered again, ensure WAKE (PS0) is held HIGH during reset for SPI selection, and only pulsed LOW for wake when sending commands.
* WAKE-on-write can remain; remove only if later proven unnecessary.
Next actions:
* Clean up debug logs, tune loop gains, and document final wiring/strap requirements in README.

Date: 2025-11-10
Context:
* With IMU streaming, the control felt too soft; servo response to large tilt changes was sluggish.
Decisions:
* Increase control loop rate and gains for stronger, faster response while keeping integral at 0 initially to avoid windup.
Changes made:
* sdkconfig.defaults:
	- CONFIG_GIMBAL_LOOP_HZ: 50 → 100
	- CONFIG_GIMBAL_ALPHA: 980 → 950 (more accel blend for quicker visible response)
	- CONFIG_GIMBAL_PID_KP: 1800 → 6000
	- CONFIG_GIMBAL_PID_KI: 0 → 0 (unchanged; keep off for now)
	- CONFIG_GIMBAL_PID_KD: 50 → 100
Risks/notes:
* Higher Kp may saturate the servo near large errors (clamped to ±105°). If oscillations appear, lower Kp to ~4000 or raise KD.
* If steady-state error remains, introduce a small Ki (e.g., 50–100 → 0.05–0.10) and add anti-windup later if needed.
Next actions:
* Rebuild/flash and evaluate step response (tilt by hand). Adjust Kp/Kd in small increments; consider enabling a small Ki once stable.

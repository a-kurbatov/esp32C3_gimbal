Verbal Explanation of the Gimbal Control Scheme

Single-axis (tilt) gimbal stabilization loop that keeps an antenna level relative to the horizon. It uses an IMU (ICM-42688-P) mounted on the base to measure platform tilt and drives a servo motor through a control loop. The system relies on factory-calibrated PWM-to-degree mapping for servo positioning, eliminating the need for an encoder. Horizon level can be user-defined via a button press for calibration. Mechanical limits are incorporated: antenna rotation capped at +/-32 degrees (with 150mm lever arm), driven by a servo with +/-105 degrees range (45mm radius gear), implying a mechanical ratio of approximately 3.28:1 (servo angle / antenna angle) for position mapping.

1. Mechanical setup

The antenna is mounted on a tilt axis whose center of mass is about 150 mm away from the rotation shaft, forming a lever arm.

A high-torque servo motor (e.g., DS3230 with 270° limited rotation, 30KG-cm torque at 6-8.4V, metal gears) with a 45 mm radius gear attached drives the shaft via a linkage or gear system, translating servo rotation to antenna tilt.

Maximum antenna rotation: +/-32 degrees.

Maximum servo rotation: +/-105 degrees (within 270° limit).

Mechanical ratio: ~3.28 (adjusted from lever radii and angle limits); scale commands accordingly.

The IMU (ICM-42688-P) is mounted on the base, sensing platform tilt and angular rate relative to gravity.

(Optional: Button calibration sets horizon offset at startup or on demand.)

2. Electronics

A microcontroller (e.g., ESP32-based board) connects via:

SPI to the IMU.

PWM output to the servo (standard 50 Hz signal for position control).

A button (digital input pin) for user-defined horizon calibration.

Power: 24 V (regulated to servo's 6-8.4V and MCU's 3.3V/5V).

3. Signal flow

IMU Data: Accel → platform tilt; gyro → motion; fused via complementary filter.

User Calibration: Button captures IMU tilt as offset (stored in EEPROM).

Sensor Fusion: Provides measured platform tilt relative to offset.

Control Loop (PID): Target = 0°; error = target + measured tilt (inverted counteraction); output antenna correction (clamped +/-32°).

Motor Command: Scale by ratio to servo angle (clamped +/-105°); map to PWM (500-2500 µs).

Feedback: Servo internal positioning; IMU enables feedforward rejection up to 80 deg/s.

4. Software structure

init() → setup SPI/IMU, PID, PWM, button; define constants (ratio, max angles).

loop() → read IMU, filter/offset/invert, PID compute, scale/clamp, update PWM, debug. At 10 Hz.

Button handler: Capture/store offset.

Tunables: PID gains, filter α, PWM min/max, ratio, limits.

5. Purpose

The system compensates for platform tilt/vibration up to 80 deg/s, maintaining antenna level to user-defined horizon with button calibration, respecting mechanical limits.

Would you like me to turn that into a short AI coding prompt (for ESP32 + 270° limited servo like DS3230, using ICM-42688-P on base, button calibration, and mechanical ratio/limits) based on this updated scheme?

Write embedded C/C++ firmware for 1-axis antenna tilt gimbal (150mm arm, +/-32° antenna max, driven by 270° servo like DS3230 with 45mm gear, +/-105° range, ratio ~3.28).
Power: 24V regulated; servo via ESP32 PWM.
Sensors: ICM-42688-P IMU (SPI, base-mounted) for platform pitch; button for horizon calibration.

Functions: Init IMU/button/PWM, define constants (ratio, max angles).
Fuse IMU with complementary filter; apply offset; invert tilt.
10Hz PID loop for level (target 0°, inverted tilt); clamp limits.
Output scaled PWM to servo (factory mapping).
Button handler: store offset (EEPROM).
Serial debug: angles/error/PID.

Structure: init(), loop(), read/filter/PID/update/calib functions.
Tunables: PID gains, α, PWM min/max, ratio/limits.
Compact, commented code.

---

AI agent journal (read first when summarizing)

- Maintain an append-only project journal at `.github/copilot-journal.md`.
- On every iteration where you make or propose changes, append a new dated entry with:
	- Context (what changed), Decisions (final choices), Changes made, Open questions/risks, Next actions.
- When asked to “summarise conversation history” or provide project status, read the journal first and base the summary on the latest entries, then add any new context from this session.
- Never rewrite or delete past entries.

---

Board targets & migration plan (must-read for AI agents)

- Phase 1: Target standard ESP32-C3 with external BNO080 / BNO085
	- Hardware: ESP32-C3 (Super Mini style board — pinout image may be present in repo). Build system: ESP-IDF (confirmed).
	- External IMU: BNO080 / BNO085 over I2C (SPI option supported by some breakout boards). Use an adapter layer so the rest of the control code is sensor-agnostic.
	- Typical things to check: I2C bus instance used by the board (`I2C_NUM_0` vs `I2C_NUM_1`), pull-up resistors on SDA/SCL, and the sensor's AD0/ADDR pin which selects I2C address (commonly 0x4A or 0x4B — verify on hardware).
	- Prefer existing, well-maintained BNO08x drivers (SparkFun / Adafruit / vendor SDK) rather than re-implementing the protocol. Confirm license compatibility before importing.
		- Implementation notes: encapsulate the sensor behind a small interface (e.g., IMU::init(), readAccel(), readGyro(), readQuat()) so swapping sensors later requires minimal changes.

- Phase 2: Migrate to CodeCell C3 (board with built-in IMU)
	- Before changing code, obtain the CodeCell C3 datasheet and identify the built-in IMU part number, bus (internal I2C/SPI), and pin mapping. The built-in IMU may use a different driver/API than the BNO08x.
	- Replace only the IMU adapter implementation; keep PID, calibration and motion-control code unchanged.
	- Test migration using recorded IMU traces or HIL tests to avoid repeated flashing.

Actionable checklist for AI contributions

1. Detect build system: ESP-IDF is the default for this project. Use `idf.py build` then `idf.py -p <PORT> flash monitor` to flash / debug. Run `idf.py set-target esp32c3` if required by your IDF version.
Hardware & wiring specifics (confirmed by user)

- Target MCU: ESP32-C3 (Super Mini). See attached pinout image for candidate pins. Before coding, confirm which pins will be used for:
 - Target MCU: ESP32-C3 (Super Mini). See attached pinout image for candidate pins. Use the concrete pin mappings below (you provided these):

	ESP32-C3 pin mappings (Super Mini labels / aliases):
	- GPIO4  = A4 = SCK
	- GPIO3  = A3
	- GPIO2  = A2
	- GPIO1  = A1
	- GPIO0  = A0
	- GPIO5  = A5 = MISO
	- GPIO6  = MOSI
	- GPIO7  = SS
	- GPIO8  = SDA  (recommend I2C SDA)
	- GPIO9  = SCL  (recommend I2C SCL)
	- GPIO10
	- GPIO20 = RX
	- GPIO21 = TX

	Suggested defaults for ESP32-C3 (confirm or change):
	- I2C: SDA = GPIO8, SCL = GPIO9 (these match the board labels)
	- Servo PWM: choose a PWM-capable pin (suggest GPIO5 or GPIO7 if not used by SPI)
	- Calibration button: choose any free GPIO with a pull-up (suggest GPIO0/A0 or GPIO1/A1 if available)

	CodeCell C3 pin mappings (user-provided):
	- GPIO2  = ADC = PWM
	- GPIO3  = ADC = PWM
	- GPIO1  = ADC = PWM
	- GPIO7  = PWM
	- GPIO6  = PWM
	- GPIO5  = PWM
	- GPIO8  = PWM = SDA
	- GPIO9  = PWM = SCL

	Suggested defaults for CodeCell C3:
	- I2C: SDA = GPIO8, SCL = GPIO9
	- Servo PWM: any PWM-capable pin (GPIO5 or GPIO6 are good choices)
	- Calibration button: any free ADC/PWM pin with input capability (recommend GPIO1 or GPIO2)

	IMU note: both boards will use the same IMU (BNO085) — adapter code will be identical and migration is straightforward (only board pin assignments change).

- Servo: DS3230PRO (spec provided)
	- Supply: 4.8 V — 6.8 V (use dedicated servo supply). PWM control: 50 Hz typical; pulse-width mapping ~500 µs .. 2500 µs maps across the servo travel (check servo datasheet and factory mapping). Support 180° and 270° modes per servo docs.

- Power input & DC-DC regulator
	- System input: 10 .. 50 V. Select a buck converter rated for that input range with an output of 6.0 V (or selectable 5–6.8 V) and sufficient continuous current for the servo (recommend >= 2–3 A headroom). The servo and MCU grounds must be common. Document the chosen module and wiring in `README.md`.

- Mechanical & calibration details
	- Hard-coded for now: lever arm = 150 mm, servo gear radius = 45 mm, antenna limit ±32°, servo usable ±105°, mechanical ratio ≈ 3.28. Make these configurable later via a JSON or sdkconfig fragment.
	- Calibration: long-press the user button to capture the current IMU pitch as the horizon offset. Store offset in NVS (ESP-IDF Non-Volatile Storage) so it persists across power cycles.

- Control & timing
	- Control loop: 10 Hz sampling and PID update (configurable). Complementary filter for accelerometer/gyro fusion; alpha is tunable.
	- Map PID output (antenna angle) → servo angle via mechanical ratio, clamp to servo limits, then convert to PWM pulse width in microseconds and send at 50 Hz.

Minimal ESP-IDF commands

- Set up environment (PowerShell / Windows):
	- Install ESP-IDF per Espressif docs and run the export script in PowerShell.
	- Build: `idf.py build`
	- Flash + monitor: `idf.py flash monitor` (or omit `-p` and let auto-detect pick the port)

Questions I still need you to answer before I scaffold code

1. Confirm exact ESP32-C3 board model (the Super Mini pinout image is helpful but please confirm SDA/SCL pin numbers and which GPIO you want reserved for servo PWM and button).
2. Confirm BNO08x comms: I2C (address used) or SPI.
3. Confirm servo model/firmware mapping if you have measured pulse→angle values, otherwise I will use 500–2500 µs as default.
4. Confirm preferred DC-DC module or whether you want me to list recommended wide-input buck modules.

If you confirm those, I will scaffold an ESP-IDF project with:
- IMU adapter interface + BNO08x adapter using an existing permissively-licensed driver
- Servo PWM module (50 Hz, µs pulse width API)
- NVS-based calibration storage and long-press handler
- A simple `main/` app with 10 Hz PID loop and serial debug

2. Add an IMU adapter interface in `src/` (or `main/`) and implement a BNO08x-backed adapter referencing an existing open-source driver.
3. Add board-specific config for ESP32-C3 (I2C pins, bus number) in a single config header or `sdkconfig` fragment.
4. When migrating to CodeCell C3, add a second adapter implementation and a board selection macro or runtime autodetect.
5. Include wiring notes and default I2C address(s) in `README.md` and this file after testing on hardware.

If you want I will: create the IMU adapter scaffold, add a BNO08x implementation that uses an existing driver, and add a CodeCell adapter stub — tell me the preferred build system and whether PlatformIO board IDs or ESP-IDF target names should be used.

---

2-axis MSP mode (new)

- Source: MSP from Betaflight 4.4.2+ at 115200 on UART1 (defaults RX=GPIO5, TX=GPIO6). Polls MSP v1 MSP_ATTITUDE at 50–100 Hz.
- Actuators: 2 servos — pitch on GPIO10 (existing), yaw on GPIO8 (new LEDC channel 1). PWM 50 Hz; default 500–2500 µs mapping.
- Target capture: long-press captures pre-flight azimuth (horizon). Controller holds that world-fixed direction in flight.
- Controller: computes world target vector [cos(az), sin(az), 0], transforms to body using assumed R = Rz(yaw) Ry(pitch) Rx(roll), then decomposes to yaw_cmd = atan2(vB.y, vB.x), pitch_cmd = atan2(-vB.z, sqrt(vB.x^2+vB.y^2)). Applies small deadband and output smoothing.
- TODOs: verify axis conventions (signs/order) and add Kconfig flips; add NVS persistence for azimuth; add per-axis mechanics/limits; MSP robustness (timeouts/resync/v2); expose gains and smoothing in Kconfig.
- Logs: monitor-parsable format — "MSP reads: roll=.. pitch=.. yaw=..; gimbal_rel: pitch=.. yaw=.." at ~2 Hz.
# Copilot 2-Axis Project Journal (append-only)

Purpose: Track decisions, changes, and open items for the MSP (2-axis) integration. Append new dated entries only. Do not edit past entries.

Entry template
- Date: YYYY-MM-DD
- Context: What changed this iteration (files, settings, hardware assumptions)
- Decisions: Finalized choices (pins, APIs, parameters)
- Changes made: Commits/edits and their intent
- Open questions / risks
- Next actions for the agent

## Entries

Date: 2025-11-13
Context:
- Kickoff MSP (2-axis) integration using Betaflight 4.4.2+ at 115200 baud via MSP v1 (initially) with MSP_ATTITUDE.
- Reuse pins GPIO5 (RX) and GPIO6 (TX) on UART1 for MSP. This build path excludes SPI IMU usage.
- Two servos will be driven: Pitch on GPIO10 (existing), Yaw on GPIO8 (new). Mechanics/ranges TBD.
Decisions:
- Add menuconfig option "MSP IMU (2-axis)", and a new MSP settings submenu.
- Hardcode initial frame/sign conventions for v1; axis customization UI to be added later.
- Long-press button captures current yaw and pitch from MSP as pre-flight targets (azimuth/horizon) to hold.
Changes made:
- Kconfig: Added GIMBAL_IMU_SELECT_MSP_2AXIS; added MSP submenu (UART port, RX/TX pins, baud); added GIMBAL_SERVO_YAW_GPIO.
- sdkconfig.defaults: Added commented selection line for MSP; set defaults for MSP UART and yaw servo pin.
Open questions / risks:
- Confirm MSP v1 MSP_ATTITUDE (0.1° units) is acceptable; add MSPv2 fallback later if needed.
- Electrical: ensure FC UART is 3.3 V TTL, non-inverted.
- Axis mapping/signs: will be hardcoded for v1; expose via Kconfig later.
Next actions:
- Step 2: Implement MSP transport + minimal parser in a new component (imu_msp) with UART task, request/response for MSP_ATTITUDE, checksum verify, and shared yaw/pitch state.

Date: 2025-11-13
Context:
- Implemented Step 2: new component imu_msp with MSP v1 polling of MSP_ATTITUDE at 50 Hz on UART1 (defaults RX=GPIO5, TX=GPIO6, 115200).
- Added API imu_msp_init/read and shared state (yaw/pitch/roll deg + age_ms). Checksum validated; values parsed from 0.1° units.
Decisions:
- Keep MSP-only build guard; imu_msp_init() is a no-op when MSP mode is not selected.
Changes made:
- components/imu_msp/: CMakeLists, imu_msp.h/.c
Open questions / risks:
- If FC outputs MSPv2 only, add v2 framing later.
Next actions:
- Step 3: Wire MSP into app: initialize imu_msp in MSP mode and, for now, log MSP yaw/pitch while keeping servos centered (control to be implemented in next steps).

Date: 2025-11-13
Context:
- Implemented Step 3: main integrates MSP mode. When MSP is selected, skip legacy IMU init; call imu_msp_init; loop reads and logs MSP yaw/pitch (2 Hz) while servos remain centered (1500 us).
Decisions:
- Leave control (dual-servo yaw+pitch) to next step; retain button long-press for future use to capture targets.
Changes made:
- main.c: conditional include of imu_msp.h; MSP initialization path; MSP loop with logging.
Open questions / risks:
- None blocking; next step will add dual-servo control and target capture.
Next actions:
- Step 4/5: Dual-servo driver extension and 2-axis control loop with long-press to set targets.

Date: 2025-11-13
Context:
- Implemented 2-axis controller in MSP mode: yaw-first, then pitch; captures pre-flight azimuth on long-press to hold horizon at that azimuth.
- Added a second servo channel (yaw) using LEDC channel 1; pitch remains on existing channel.
- Implemented world-vector approach: transform target azimuth vector into body frame using MSP yaw/pitch/roll and decompose to yaw/pitch commands.
- Logging format updated for monitor parsing: "MSP reads: roll=.. pitch=.. yaw=..; gimbal_rel: pitch=.. yaw=..".
Decisions:
- Leave axis conventions as TODO with explicit comments; add Kconfig sign flips later after hardware validation.
Changes made:
- main.c: added LEDC yaw channel, long-press capture for target azimuth, world->body transform, yaw/pitch decomposition, smoothing and deadband, and compact logs.
Open questions / risks:
- Axis sign and rotation order may need flips depending on FC conventions; add Kconfig to adjust.
- Servo mechanics per axis (min/max/ratio/limits) still TODO; using shared limits for now.
Next actions:
- Persist target azimuth to NVS; add Kconfig for signs, gains, deadband/smoothing; refactor dual-servo driver; improve MSP robustness.

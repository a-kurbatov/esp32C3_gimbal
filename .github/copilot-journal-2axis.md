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

Date: 2025-11-13
Context:
- MSP bring-up refinements: removed consuming RX sniff/echo drains to prevent header loss; added UART RX flush at init; optional buffered RX debug; throttled raw logs.
Decisions:
- Keep poll rate; reduce monitor noise; prefer non-consuming diagnostics.
Changes made:
- imu_msp.c: removed echo drain; switched to uart_get_buffered_data_len when debug enabled; added uart_flush_input at init.
Open questions / risks:
- If FC enforces MSPv2 only, add v2 parsing later.
Next actions:
- Validate sustained MSP updates and tune read timeouts if needed.

Date: 2025-11-14
Context:
- Yaw heading interpretation clarified: yaw is 0..359 deg (heading at boot), not 0.1°.
- Added persistent raw ATT debug (throttled to ~2 Hz) and 2 Hz main log: raw ints + gimbal_rel.
Decisions:
- Use yaw directly in degrees; roll/pitch remain deci-deg (÷10).
Changes made:
- imu_msp.c: yaw scaling changed to use raw degrees; raw ATT logs include ints and deg; main log prints "imu_msp: raw ints r=.. p=.. y=..; gimbal_rel: pitch=.. yaw=.." at 2 Hz.
Open questions / risks:
- None blocking.
Next actions:
- Proceed to yaw control logic improvements.

Date: 2025-11-17
Context:
- Reworked MSP yaw control to be servo-centric and robust at 0/359 seam.
- Added calibration of yaw center (long-press); stored in NVS; loaded on boot.
- Added Kconfig for yaw mapping: total yaw servo travel, yaw gearing (servo/antenna), antenna opening.
- Implemented unwrapped nearest-branch selection, 1 s phase flip animation with 1.5 s cooldown and near-limit hysteresis; antenna overlap smooth deadband.
Decisions:
- Center the mapping on servo center; always pick the branch that keeps servo within half-swing; only flip when sustained near the limit.
Changes made:
- Kconfig: GIMBAL_YAW_SERVO_MAX_DEG, GIMBAL_YAW_RATIO_SERVO_PER_ANT_X1000, GIMBAL_ANTENNA_OPENING_DEG.
- main.c (MSP path): base error uses yaw_center; apply smooth deadband; compute servo_cmd via yaw_ratio; clamp to half-swing; pick e0±360 branch closest to previous command; flip animation when needed; yaw-dependent pitch gain retained (cos(|yaw_cmd|)).
- NVS: new key yaw_ctr; load/save helpers; long-press captures yaw center.
Build fixes:
- Included esp_timer.h; removed duplicate beam_half_deg and stray state from UART-RVC block.
Open questions / risks:
- Optional asymmetry (e.g., right 270, left 90) can be reintroduced via config; current servo-centric default is symmetric limits from half-swing.
Next actions:
- Optionally expose flip timing/cooldown/hysteresis in Kconfig; allow asymmetric left/right limits; tune yaw->pitch taper curve; document yaw calibration in README.

Date: 2025-11-18
Context:
- Safety constraint added to protect mechanics: configurable max allowed yaw servo angle distinct from physical/nominal travel. Controller clamps commands to the safe window.
Decisions:
- Keep servo-centric mapping but enforce a separate "allowed" envelope so destructive end-stop rotations are avoided.
Changes made:
- Kconfig: added GIMBAL_YAW_SERVO_ALLOWED_DEG (total allowed deg; controller clamps to ±allowed/2). Defaults added in sdkconfig.defaults.
- main.c (MSP path): compute servo_half_allowed_deg; derive gimbal limits from allowed half-swing and yaw gearing; clamp yaw_servo to ±servo_half_allowed_deg; write yaw with these limits.
Open questions / risks:
- Consider asymmetric allowed windows per direction if needed (e.g., +larger to right, smaller to left).
Next actions:
- Expose flip timing/cooldown/hysteresis in Kconfig; optionally add asymmetric yaw limits; update README with safety guidance and calibration steps.

Date: 2025-11-18
Context:
- Rewrote MSP yaw logic from scratch to use a simple 0..359° heading loop with calibration-based forward, continuous optimal-vector computation, and bounded flip animation only when beyond limit + half beam.
- Added explicit servo-direction logging to the 2 Hz line to show left/right and magnitude with limits.
Decisions:
- Keep existing Kconfig settings (allowed servo angle, yaw ratio, antenna opening) and reuse pitch path; center servo at boot and treat MSP 0° as default forward.
- Start transitions only when they reduce antenna error considering beam overlap; cap flip speed to ~1 s per 360° with cooldown; cancel if operator backs off.
Changes made:
- main.c (MSP section): removed nearest-branch heuristics and phase_* state; added branch_offset_deg with flip_active animation; limit checks use antenna-side limits derived from servo allowed half-swing and gearing; continuous recompute during animation.
- Logging: appended "servo=%+X (left/right Y) lim=±Z" to gimbal_rel output.
Open questions / risks:
- Desired exact log wording for servo info (example given shows "servo=+97 (-120)"); adjust formatting if needed.
- Verify real FC yaw range and sign; adjust wrap/signs in Kconfig if conventions differ.
Next actions:
- Field-test flips at both ends; tune flip_duration/cooldown and beam overlap; finalize log format; optionally expose flip parameters via Kconfig.

# Copilot To-Do (2-axis MSP integration)

This is an append-only checklist. Do not delete past items; strike-through when completed.

## High priority
- [ ] Axis conventions: confirm MSP roll/pitch/yaw signs and rotation order; add Kconfig flips for yaw/pitch/roll signs and optional R order.
- [ ] Persist MSP target azimuth to NVS on long-press; load at boot; add reset path.
- [ ] Servo mechanics per axis (TODO): yaw vs pitch limits, ratio, min/max pulse; expose via Kconfig; document in README.
- [ ] Dual-servo module: refactor to a shared servo driver with multiple channels (avoid duplicating LEDC setup).
- [ ] MSP robustness: timeouts, resync on framing error, optional MSP v2 support; configurable poll rate.
- [ ] Menuconfig UX: expose deadband/smoothing per axis; expose gains per axis; expose yaw wrap handling (shortest-path).
- [ ] README update: MSP wiring (UART1 RX=GPIO5, TX=GPIO6), servo pins (pitch=GPIO10, yaw=GPIO8), and FC config (Betaflight 4.4.2+, 115200, MSP enabled).

## Medium priority
- [ ] Optional PI/PID per axis (now we smooth commands; add per-axis P/PI/PID with anti-windup if Ki>0).
- [ ] Optional roll feedforward (map roll into pitch if needed for mechanical coupling/stability).
- [ ] Telemetry/logging: add structured logs and/or a simple status overlay; Kconfig to enable/disable.
- [ ] Unit tests or bench mode: stub MSP frames for offline verification.
- [ ] Error LEDs or on-screen indicators for MSP link loss.

## Low priority / future work
- [ ] Axis customization UI (runtime), including mapping and sign flips without rebuild.
- [ ] Continuous-yaw support (if hardware allows) or soft-limit taper near extremes.
- [ ] Motion profile (slew limit) to avoid fast snaps.
- [ ] Calibration routines for servo center/scale; optional teach mode.
- [ ] GitHub Actions CI build matrix for MSP and IMU modes.

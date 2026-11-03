# Tacho Config

Walks through configuring the speed loop (`DCMotorTacho`) for your motor. **Prerequisite:** complete `servo_config/` step 04 first — every RPM reading depends on the calibrated CPR.

## Procedure

0. Carry over from servo_config: the empirical fudge factor / CPR (step 04). Carry your filled-in `baseSpecs.h` forward from servo_config (it holds `PPR`, `GEAR_RATIO`, `EMPIRICAL_FUDGE_FACTOR`, and the rest) — an uncalibrated CPR makes every RPM reading proportionally wrong.
1. **speedInterval**: the speed loop's update period (`SPEED_INTERVAL`, default 20 ms). The speed is measured over the last ~50 ms of samples (at most 10) whatever the interval, so a short interval reacts faster without losing low-speed resolution. As a rule of thumb keep `CPR × RPM_min / 60000 × 50 ≳ 4` counts.
2. **Feedforward and gains**: the `01_tune_speed_pid` sketch: an open-loop PWM sweep for the feedforward, then the gains. See its README.

## Sample Results

- 26PG-3429-19-EN DC motor (PPR 12, gear 19:1, fudge 0.875)

| Parameter | L298N | LMD18200 |
| --- | --- | --- |
| SPEED_INTERVAL | | |
| Feedforward pwmStatic, kv | | |
| Speed Kp, Ki | | |

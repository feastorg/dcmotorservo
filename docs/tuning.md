# Tuning Guide

Tuning has three stages: determine your encoder's CPR, configure the position controller (`DCMotorServo`), then — if using speed control — configure the speed loop (`DCMotorTacho`).

The `examples/servo_config/` and `examples/tacho_config/` sketches are designed to walk through each stage interactively.

---

## Stage 1 — Determine CPR (Counts Per Revolution)

CPR is the number of encoder counts for one full output shaft revolution **after the gearbox**, including the quadrature ×4 factor and any empirical correction.

**Formula:**

```
CPR = PPR × GEAR_RATIO × 4 × EMPIRICAL_FUDGE_FACTOR
```

| Value                    | Source                                    |
| ------------------------ | ----------------------------------------- |
| `PPR`                    | Encoder datasheet (pulses per revolution) |
| `GEAR_RATIO`             | Motor/gearbox datasheet                   |
| `4`                      | Quadrature decoding multiplier (fixed)    |
| `EMPIRICAL_FUDGE_FACTOR` | Measured — see below (start at `1.0`)     |

**To measure the fudge factor:**

1. Mark the output shaft.
2. Command `moveTo(CPR)` (one revolution) with fudge = 1.0.
3. Measure the actual angle. If it turned 370° instead of 360°, fudge = 360/370 ≈ 0.973.
4. Iterate until one revolution is accurate.

Only needed for `DCMotorTacho`. `DCMotorServo` works purely in raw counts.

---

## Stage 2 — Tune the Position Loop (DCMotorServo)

### 2a — Find PWM Skip

`setPWMSkip(value)` sets the minimum PWM added to every non-zero output so the motor overcomes static friction and actually moves.

1. Open `examples/servo_config/01_pwm_skip/`.
2. Slowly increase `MIN_PWM` until the motor reliably starts from rest.
3. Typical range: 10–60 depending on motor and gearbox.

### 2b — Set Accuracy

`setAccuracy(counts)` is the dead-band: when `|setpoint - encoder| < accuracy` the motor is considered done and PWM drops to zero.

- Too small → motor hunts (oscillates around target).
- Too large → coarse positioning.
- Start with ~1–2% of CPR. Tighten after PID is tuned.

### 2c — Tune PID Gains

Use `examples/servo_config/03_tune_servo_pid/` with the Arduino Serial Plotter (call `servo.getSerialPlotter()` every ~20 ms).

**Procedure:**

| Step | Action                                                                                  |
| ---- | --------------------------------------------------------------------------------------- |
| 1    | Set Ki = 0, Kd = 0. Increase Kp until the motor reaches the target without oscillating. |
| 2    | If there is steady-state error, increase Ki in small steps.                             |
| 3    | If the motor overshoots, increase Kd to dampen.                                         |
| 4    | Iterate: small Kd increase often lets you raise Kp further.                             |

**Typical starting point:** `Kp=0.15, Ki=0.0, Kd=0.001`

---

## Stage 3 — Tune the Speed Loop (DCMotorTacho)

Only needed if using `DCMotorTacho`. It drives PWM directly: feedforward from the target speed, plus a PI correction on the measured speed. The position loop's gains don't affect it.

### 3a — Set speedInterval

The default is 50 ms. The speed is measured over the last ~50 ms of samples (at most 10) whatever the interval, so a shorter interval (5–20 ms) reacts faster without losing resolution. Keep `CPR × RPM_min / 60000 × 50 ≳ 4` counts; for slower motors use a longer interval.

### 3b — Feedforward

Use `examples/tacho_config/01_tune_speed_pid/` (`PWM=` command), with the motor loaded as in use:

| Step | Action |
| ---- | ------ |
| 1 | Drive open loop at four or five PWM values across your range and note the steady counts/s at each. |
| 2 | Fit counts/s = `kv` × (PWM − `pwmStatic`). Repeat in reverse and take the lower `kv` and the higher `pwmStatic`. |
| 3 | `tacho.setFeedforward(pwmStatic, kv)` and `tacho.setTrimLimit(20)`. |

Err low. Feedforward that under-drives costs a little start-up time; in a host model, feedforward 20 % too high overshot a 60 RPM start by about 15 % (200 ms mean). `kv` scales with supply voltage, so calibrate at the supply you run at.

### 3c — Gains

Plot `getMeasuredSpeedRPM()` against `getDesiredSpeedRPM()`.

| Step | Action |
| ---- | ------ |
| 1 | Start from the defaults, Kp 0.6 and Ki 3.0 (PWM per RPM, per RPM-second). |
| 2 | Raise Kp until the speed tracks briskly with slight oscillation, then back off. |
| 3 | Ki removes the remaining error; too much shows as slow hunting. |
| 4 | Grip the shaft briefly and release: it should return to speed without a lunge. |

Without feedforward the loop is a plain PI on PWM. It works, but the integral has to build the whole drive, so it is slower to reach speed.

---

## Quick Reference — Default Values

| Setting           | Default     | Setter                         |
| ----------------- | ----------- | ------------------------------ |
| PWM skip          | 50          | `setPWMSkip(value)`            |
| Max PWM           | 255         | `setMaxPWM(value)`             |
| Position accuracy | 30 cts      | `setAccuracy(value)`           |
| Position Kp/Ki/Kd | 0.1/0.2/0.1 | `servo.setPIDTunings(...)`     |
| PID sample time   | 50 ms       | `servo.myPID->SetSampleTime()` |
| Speed interval    | 50 ms       | `DCMotorTacho` constructor     |
| Speed Kp/Ki       | 0.6/3.0     | `tacho.setPIDTunings(...)`     |
| Feedforward       | none        | `tacho.setFeedforward(...)`    |
| Trim limit        | 255         | `tacho.setTrimLimit(...)`      |

# API Reference

## DCMotorServo

### Constructor

```cpp
DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake,
             EncoderReadFunc eRead, EncoderWriteFunc eWrite)
```

Provide four function pointers that adapt your motor driver and encoder library to the control interface. See [getting-started.md](getting-started.md) for the required signatures.

**Defaults set in the constructor:**

| Parameter         | Default       | Description                        |
| ----------------- | ------------- | ---------------------------------- |
| PWM skip          | 50            | Minimum PWM to overcome stiction   |
| Position accuracy | 30            | Acceptable error in encoder counts |
| Max PWM           | 255           | Upper PWM clamp                    |
| PID (Kp, Ki, Kd)  | 0.1, 0.2, 0.1 | Starting tuning values             |
| PID sample time   | 50 ms         |                                    |

---

### Public Member

```cpp
PID *myPID
```

Direct access to the underlying [Arduino PID library](https://github.com/br3ttb/Arduino-PID-Library) object for advanced configuration (e.g. `myPID->SetSampleTime()`).

---

### Motion

#### `run()`

```cpp
void run()
```

Reads the encoder, computes PID output, and sends the result to the motor driver. **Call every loop iteration.** With the extrema features active, `run()` first services the special modes: while homing it drives the homing move instead of the PID (encoder input is not tracked into the PID); while a stall fault is latched it does nothing (motor stays braked); while the target lies beyond a triggered endstop it holds the motor braked and pulls the target to the held position.

Inside the dead-band (`setAccuracy()`) there is no drive and the PID is not computed, so its integral is held for the next move rather than reset. Outside it, a drive whose direction opposes the error is dropped to 0: near zero error a small output of the wrong sign would otherwise become a `pwmSkip` kick away from the target.

#### `stop()`

```cpp
void stop()
```

Engages the brake, suspends the PID controller, and syncs the target to the current position so the motor holds where it stopped rather than chasing the old setpoint. Also cancels an in-progress homing move.

#### `drive(pwm)`

```cpp
void drive(int16_t pwm)
```

Drives the motor at a signed PWM directly, bypassing the position PID; `DCMotorTacho` uses it for speed mode. Call it every pass in place of `run()`. The target follows the shaft, so a later `run()` holds where the motor is. Stall detection, attached endstops and software travel limits apply as in `run()`: a stall latches, and an endstop or limit in the direction of drive brakes the motor. Clamped to ±`maxPWM`; does nothing while homing or while a stall is latched.

#### `moveTo(position)`

```cpp
void moveTo(long position)
```

Sets an absolute target position in encoder counts.

#### `move(delta)`

```cpp
void move(long delta)
```

Adds `delta` encoder counts to the current target (relative move).

---

### Status

#### `finished()`

```cpp
bool finished()
```

Returns `true` when the encoder is within the accuracy window of the target and the PWM output is zero. Always `false` while a homing move is in progress. A target blocked by a triggered endstop is pulled to the held position, so `finished()` reports `true` once the motor has gone as far as it can.

#### `getActualPosition()`

```cpp
long getActualPosition()
```

Returns the current encoder count.

#### `getRequestedPosition()`

```cpp
long getRequestedPosition()
```

Returns the current PID setpoint (target count).

---

### Configuration

#### `setPIDTunings(Kp, Ki, Kd)`

```cpp
void setPIDTunings(double Kp, double Ki, double Kd)
```

Sets the PID gains. See [tuning.md](tuning.md).

#### `setPWMSkip(range)`

```cpp
bool setPWMSkip(uint8_t range)
```

Sets the minimum PWM added to the PID output to overcome static friction. Determined experimentally. Returns `false` if `range >= 255`.

#### `setMaxPWM(maxPWM)`

```cpp
void setMaxPWM(uint8_t maxPWM)
```

Clamps the maximum PWM output. Also updates PID output limits accordingly.

#### `getMaxPWM()`

```cpp
uint8_t getMaxPWM()
```

Returns the maximum PWM output.

#### `setAccuracy(range)`

```cpp
void setAccuracy(unsigned int range)
```

Sets the position dead-band in encoder counts. The motor is considered finished when error is below this value.

#### `setCurrentPosition(position)`

```cpp
void setCurrentPosition(long position)
```

Resets the encoder count to `position` and updates the PID input. Useful for zeroing after homing.

---

### Extrema sensing

Three tiers, usable independently or together: software travel limits, physical
endstops, and encoder-based stall detection. A non-blocking homing move ties
them together by zeroing the encoder at a detected extreme.

#### `setTravelLimits(min, max)`

```cpp
void setTravelLimits(long min_position, long max_position)
```

Sets software extrema in encoder counts. Targets passed to `move()`/`moveTo()` are clamped into `[min, max]` (arguments are swapped if given out of order). The current setpoint is clamped immediately.

#### `clearTravelLimits()`

```cpp
void clearTravelLimits()
```

Removes the software travel limits.

#### `attachEndstops(minStop, maxStop)`

```cpp
void attachEndstops(EndstopReadFunc minStop, EndstopReadFunc maxStop)
```

Attaches physical endstop sensors (limit switch, hall effect, ...). Each is a `bool ()` function returning `true` when triggered; pass `nullptr` for a side with no sensor. While the target position lies beyond a triggered endstop, `run()` holds the motor braked and pulls the target to the held position — nothing (including `DCMotorTacho`'s speed loop) can wind the setpoint past the stop, and a later switch release cannot cause an uncommanded move. Motion away from the endstop remains allowed; command a new target to move off the stop. The hold is judged on the direction of the position error, so it is stable — the motor does not chatter against the switch.

#### `enableStallDetection(timeout_ms, min_counts)`

```cpp
void enableStallDetection(unsigned long timeout_ms, long min_counts = 4)
```

Enables encoder-based stall detection: if the motor is being driven but the encoder advances fewer than `min_counts` counts within `timeout_ms`, the motor is braked and a stall fault is latched. During homing, a stall instead marks the extreme (sensorless homing). Choose `timeout_ms` longer than your mechanism's worst-case time to produce `min_counts` at minimum PWM.

#### `disableStallDetection()`

```cpp
void disableStallDetection()
```

Disables stall detection.

#### `isStalled()` / `clearStall()`

```cpp
bool isStalled()
void clearStall()
```

`isStalled()` reports a latched stall fault; while latched, `run()` keeps the motor braked. `clearStall()` clears the fault and resumes normal operation — note the target position is unchanged, so the motor will drive toward it again; re-target first if the cause of the stall is still present.

#### `startHoming(direction, pwm, max_travel)`

```cpp
bool startHoming(int8_t direction, uint8_t pwm, long max_travel = 0)
```

Starts a non-blocking homing move: drives at a fixed `pwm` (clamped to `[pwm_skip, maxPWM]`) toward an extreme — negative `direction` for the minimum side, positive for the maximum side — until the matching endstop triggers or a stall is detected, whichever is attached/enabled. Software travel limits are bypassed during the move, and any latched stall fault is cleared (a stall during homing marks the extreme, not a fault). On success the motor brakes, the encoder is zeroed, and the setpoint is set to the (limit-clamped) zero. `max_travel` is a failsafe bound in encoder counts (e.g. against a dead endstop switch): exceeding it aborts the move *unhomed*, holding the current position; `0` means unbounded. Returns `false` if `direction` is `0` or if neither the matching endstop nor stall detection is available to detect the extreme. Progressed by `run()`; cancel with `stop()`.

#### `isHoming()` / `isHomed()`

```cpp
bool isHoming()
bool isHomed()
```

`isHoming()` returns `true` while a homing move is in progress; poll it after `startHoming()` to detect the end of the move. `isHomed()` reports whether the last homing move actually *succeeded* (encoder zeroed at a detected extreme) — it stays `false` when homing was cancelled by `stop()`, aborted by `max_travel`, or lost its termination conditions mid-move. `isHoming() == false` alone does not imply success; check `isHomed()`.

---

### Diagnostics

#### `getDebugInfo()`

```cpp
String getDebugInfo()
```

Returns a multi-line string with encoder position, PID setpoint/input/output, PWM skip, accuracy, travel limits (when set), stall/homing state, and PID gains.

#### `getSerialPlotter(id)`

```cpp
String getSerialPlotter(int id = -1)
```

Returns a comma-separated `Setpoint:x, Input:x, Output:x` string formatted for the Arduino Serial Plotter. Pass an integer `id` when plotting multiple motors to differentiate traces.

---

## DCMotorTacho

Closed-loop speed control. Every speed interval it measures the shaft's speed and drives the motor through `DCMotorServo::drive()` with

```text
pwm         = feedforward + trim
feedforward = sign(target) * pwmStatic + target_counts_per_s / kv
trim        = Kp * error + integral, bounded to +-trimLimit
```

where `error` is the target RPM less the measured RPM. The integral doesn't accumulate while the trim or the output is at its limit in the error's direction (conditional integration), so a held shaft banks at most `trimLimit` of drive and doesn't lunge on release. With no feedforward set (the default) it is a plain PI on PWM.

The speed is measured over the last ~50 ms of samples (at most 10), so low speeds resolve to more than a count or two. Stall detection, endstops and travel limits on the servo apply in speed mode as in position mode.

### Constructor

```cpp
DCMotorTacho(DCMotorServo *servo, double cpr, unsigned long speedInterval = 50)
```

| Parameter       | Description                                                                            |
| --------------- | -------------------------------------------------------------------------------------- |
| `servo`         | Pointer to an existing, configured `DCMotorServo` object                               |
| `cpr`           | Encoder counts per revolution (after gearbox + quadrature). See [tuning.md](tuning.md) |
| `speedInterval` | Speed loop update interval in ms (default 50 ms)                                       |

---

### Methods

#### `run()`

```cpp
void run()
```

Runs the speed loop. **Call every loop iteration.** While the wrapped servo is homing or stall-latched, the speed loop is held and the servo's `run()` progresses the homing move or holds the brake; speed control resumes automatically afterwards. The zero-speed branch doesn't cancel a homing move (an explicit `stop()` does).

#### `stop()`

```cpp
void stop()
```

Brakes the motor and resets the speed loop.

#### `setSpeedRPM(rpm)`

```cpp
void setSpeedRPM(double rpm)
```

Sets the desired shaft speed in RPM. Negative values reverse direction. `0` engages the brake and resets the integral. A non-zero speed after `0` starts speed measurement afresh, so travel while the loop was idle doesn't count as speed.

#### `getDesiredSpeedRPM()`

```cpp
double getDesiredSpeedRPM() const
```

Returns the commanded RPM.

#### `getMeasuredSpeedRPM()`

```cpp
double getMeasuredSpeedRPM() const
```

Returns the most recently measured RPM, over the last ~50 ms. It is measured while the setpoint is 0 as well, but not while the servo is homing or stall-latched.

#### `setPIDTunings(Kp, Ki)`

```cpp
void setPIDTunings(double Kp, double Ki)
```

Sets the speed loop's gains: `Kp` in PWM per RPM of error (default 0.6), `Ki` in PWM per RPM-second (default 3.0).

#### `setFeedforward(pwmStatic, kv)`

```cpp
void setFeedforward(double pwmStatic, double kv)
```

Sets the feedforward: the drive the motor needs to turn at the target speed with no correction. From an open-loop PWM sweep, fit counts/s = `kv` × (PWM − `pwmStatic`): `pwmStatic` is the PWM that just starts the shaft turning, `kv` the counts/s per PWM step above it. `kv` 0 (the default) leaves only the static term. Err low: feedforward that under-drives costs a little start-up time, while over-drive overshoots at start-up. See [tuning.md](tuning.md).

#### `setTrimLimit(pwm)`

```cpp
void setTrimLimit(double pwm)
```

Bounds the correction (P + I) around the feedforward, in PWM. With feedforward set, about 20 keeps a held shaft from banking more than that. Default 255: no bound beyond `maxPWM`.

#### `getServo()`

```cpp
DCMotorServo *getServo() const
```

Returns the underlying `DCMotorServo` pointer for direct access.

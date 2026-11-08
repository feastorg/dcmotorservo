# Changelog

All notable changes to this library are documented here. Follows
[Keep a Changelog](https://keepachangelog.com) and
[Semantic Versioning](https://semver.org). History before 1.0.1 was
reconstructed from the git log.

## [Unreleased]

### Fixed

- A long position move could stop short of the target and stay there.
  The PID's integral wound up while the drive was saturated; when the
  shaft coasted past the target it pointed away from it, `run()`
  refused to drive away, and the motor sat outside the deadband with no
  drive until the integral unwound. On a Slice_DCMT bench a
  7282-count move stopped 48 counts past the target for over 60 s.
  `run()` now zeroes the integral while the drive is saturated.

## [2.0.0] — 2026-11-02

Checked by host tests and in Slice_DCMT's host motor model only, not yet
on hardware. Speed gains from 1.x don't carry over: see `docs/tuning.md`.

### Changed

- **Breaking:** `DCMotorTacho` drives PWM directly instead of moving the
  position target (#26). Every speed interval it sets PWM = feedforward
  + trim, where the feedforward is `sign(target) * pwmStatic +
  target_counts_per_s / kv` and the trim is a PI correction on the
  measured speed, bounded to ±`trimLimit`. The integral doesn't
  accumulate while the trim or the output is at its limit. In a host
  model of a 24.3 counts/s-per-PWM motor, 120 RPM now holds within
  0.1 RPM about 0.2 s after the command, where 1.2.1 took 9.7 s and then
  climbed to 159 and collapsed to 73, repeatedly; a shaft held for
  0.5 s at 120 RPM returns to 120.6 RPM rather than 150.8.
  - `setPIDTunings(Kp, Ki)` takes two gains, in PWM per RPM and per
    RPM-second (defaults 0.6 and 3.0). Speed gains from 1.x don't carry
    over.
  - New `setFeedforward(pwmStatic, kv)` and `setTrimLimit(pwm)`. With
    neither set, the loop is a plain PI on PWM.
  - `setMaxLead()` is removed: there is no position target to cap.
    `setTrimLimit()` bounds what a held shaft can bank (#24).
  - The speed is measured over the last ~50 ms of samples (at most 10),
    not one interval.
- `DCMotorServo::run()` no longer resets the PID's integral inside the
  dead-band: the PID isn't computed there, so the integral is held for
  the next move (#26).

### Added

- `DCMotorServo::drive(pwm)`: drive at a signed PWM with the position PID
  bypassed, with stall detection, endstops and travel limits applied as in
  `run()`. `DCMotorTacho` uses it.
- `DCMotorServo::getMaxPWM()`.

### Fixed

- `DCMotorServo::run()` never drives away from the target. Near zero error
  a small PID output of the wrong sign became a full `pwmSkip` kick
  backwards (feastorg/Slice_DCMT#62, #105).

## [1.2.1] — 2026-11-01

#18 is checked by host test only.

### Changed

- The repository moved to
  [feastorg/dcmotorservo](https://github.com/feastorg/dcmotorservo). The
  old GitHub URL redirects, and the PlatformIO package name is unchanged.

### Fixed

- `DCMotorTacho::getMeasuredSpeedRPM()` no longer keeps the last running
  speed while the setpoint is 0. The speed is still measured then, so it
  reads 0 once the shaft stops (#18)

## [1.2.0] — 2026-10-23

#11, #12 and #13 were bench-checked at `cc8bad6` on a Slice_DCMT gen2
board (Nano Every) on 2026-10-22, and a 300-count `setMaxLead()` removed
the lunge after a held stall
([record](https://github.com/feastorg/Slice_DCMT/blob/main/docs/bench/2026-10-22-applied-pwm-and-library-fixes.md)).
#19 is checked by host test only.

### Added

- `DCMotorTacho::setMaxLead()` caps how far the speed loop can drive the
  position target ahead of the encoder, so a stalled shaft can't bank
  travel for a lunge on release. Off by default (#13)

### Fixed

- `setAccuracy()` no longer wraps values above 255: the tolerance is now
  stored as `unsigned int`, matching the parameter (#11)
- `run()` and `finished()` compare the position error with `fabs()`. On
  the megaavr core (Nano Every) `abs()` converted it to a 16-bit `int`, so
  an error near a multiple of 65,536 counts read as inside the deadband
  and the motor was not driven (#19)
- `DCMotorTacho`: restarting the speed loop after `setSpeedRPM(0)`,
  `stop()` or a homing/stall hold no longer seeds the speed PID from the
  last output and input, which could drive the motor the wrong way (#12)
- `DCMotorTacho`: the first speed sample after starting from zero no longer
  counts the travel since the loop last ran, such as a position move, as
  speed (#13)

### Changed

- `DCMotorServo::stop()` now syncs the target to the current position in
  every case, not only when cancelling homing. A `run()` after `stop()`
  holds instead of resuming the old move, and a speed loop started after
  a position move no longer inherits its target (#13)

## [1.1.0] — 2026-07-16

Verified by code review and compile checks; hardware bench validation
is tracked in
[#3](https://github.com/feastorg/dcmotorservo/issues/3).

### Added

- Extrema sensing on `DCMotorServo` (closes #1):
  - software travel limits — `setTravelLimits()` / `clearTravelLimits()`
    clamp `move()`/`moveTo()` targets in encoder counts
  - physical endstops — `attachEndstops()` callbacks; while the target lies
    beyond a triggered stop, `run()` holds the motor braked and pulls the
    target to the held position (stable hold, no chatter, no lunge on
    switch release)
  - encoder-based stall detection — `enableStallDetection()` /
    `disableStallDetection()` / `isStalled()` / `clearStall()`; latched
    fault when the motor is driven but the encoder does not advance
  - non-blocking homing — `startHoming(direction, pwm, max_travel)` /
    `isHoming()` / `isHomed()`; terminates on the matching endstop or a
    stall (sensorless), zeroes the encoder at the extreme; `max_travel`
    failsafe aborts unhomed on a dead switch
- `l298n_homing` usage demo; "Extrema sensing" section in `docs/api.md`.

### Changed

- `stop()` also cancels an in-progress homing move, syncing the target to
  the current position.
- `DCMotorTacho::run()` holds its speed loop while the wrapped servo is
  homing or stall-latched (no setpoint windup against a frozen encoder, no
  silent homing cancellation), resuming automatically afterwards.

### Fixed

- `finished()` PWM check was vacuous (`_PWM_output` was never written in
  `run()`); now updated each cycle and homing-aware.
- Zero PID output was mapped to `+pwm_skip` drive instead of no drive
  (jolt after `stop()`).
- Constructor initializer-list order (`-Wreorder` warning).

### Docs

- README rewritten for the driver-agnostic API (the old pin-based
  sections were stale).
- Example driver-library dependencies (L298N, LMD18200) documented.
- Tuning walkthrough READMEs: servo PID (03), rotational/CPR-fudge
  calibration (04), tacho procedure incl. speedInterval selection, and
  speed-PID tuning.
- Config sketches refactored around a per-sketch `baseSpecs.h` spec
  sheet (driver, pins, motor specs, determined values) with a
  documented fill-in-and-copy-forward workflow; drifted tuning values
  unified to the reference rig's.
- `TODO.md` and the README TODO section retired; all outstanding work
  tracked as GitHub issues.

## [1.0.1] — 2026-03-15

First published release (PlatformIO registry: `cameronbrooks11/DCMotorServo`).

### Added

- PlatformIO manifest (`library.json`) and registry publication.
- Docs suite: `getting-started.md`, `tuning.md`, `api.md`.

## [1.0.0] — 2026-02

Driver-agnostic rewrite of the original pin-based library.

### Changed

- Hardware interface replaced with four function pointers
  (`MotorWriteFunc`, `MotorBrakeFunc`, `EncoderReadFunc`,
  `EncoderWriteFunc`) — any driver/encoder library works without
  touching the control code.

### Added

- `DCMotorTacho`: cascaded speed-control (RPM) loop wrapping the
  position loop.
- Guided configuration examples (`servo_config` 01–04, `tacho_config`)
  and driver-specific usage demos (L298N, LMD18200).

## Pre-1.0

Fork-and-continuation of
[julester23/DCMotorServo](https://github.com/julester23/DCMotorServo)
(pin-based API, PID + Encoder feedback), inspired by AccelStepper.

# Getting Started

## Installation

### PlatformIO

Add to `platformio.ini`:

```ini
lib_deps =
    CameronBrooks11/DCMotorServo@^1.0.1
```

Dependencies (`PID` and `Encoder`) are declared in `library.json` and will be resolved automatically.

### Arduino IDE

1. Download or clone the repository and place the folder in your Arduino libraries directory.
2. Install dependencies from Library Manager: **PID** by Brett Beauregard and **Encoder** by Paul Stoffregen.
3. Restart the Arduino IDE.

### Example driver libraries

The bundled examples drive their motors through two small driver libraries (not needed by DCMotorServo itself — your own `MotorWriteFunc`/`MotorBrakeFunc` wrappers replace them in real projects):

| Library | Provides | Install |
| --- | --- | --- |
| [LMD18200](https://github.com/feastorg/lmd18200) | `LMD18200.h` | PIO: `cameronbrooks11/LMD18200`, or clone into your libraries dir |
| [L298N](https://github.com/feastorg/l298n) | `L298N.h` | PIO: `cameronbrooks11/L298N`, or clone into your libraries dir |

---

## Concept

DCMotorServo is **driver-agnostic**. Instead of hard-coding pin logic, you supply four function pointers that match your motor driver and encoder library:

| Pointer type       | Signature                | Purpose                             |
| ------------------ | ------------------------ | ----------------------------------- |
| `MotorWriteFunc`   | `void fn(int16_t speed)` | Send signed PWM to the motor driver |
| `MotorBrakeFunc`   | `void fn()`              | Engage the driver's brake           |
| `EncoderReadFunc`  | `long fn()`              | Return current encoder count        |
| `EncoderWriteFunc` | `void fn(long pos)`      | Reset/zero the encoder count        |

This lets you use any H-bridge (L298N, LMD18200, DRV8833, …) and any encoder library without changing the control code.

---

## Minimal Example — Position Control (LMD18200)

```cpp
#include <Encoder.h>
#include <LMD18200.h>
#include <DCMotorServo.h>

LMD18200 driver(6, 7, 8);          // pwmPin, dirPin, brakePin
Encoder  enc(2, 3);

void motorWrite(int16_t s) { driver.write(s); }
void motorBrake()           { driver.brake(); }
long encRead()              { return enc.read(); }
void encWrite(long p)       { enc.write(p); }

DCMotorServo servo(motorWrite, motorBrake, encRead, encWrite);

void setup() {
    driver.begin();
    servo.setPWMSkip(25);       // minimum PWM to overcome stiction
    servo.setAccuracy(10);      // acceptable position error in counts
    servo.setPIDTunings(0.15, 0.0, 0.001);
    servo.moveTo(500);          // move to encoder count 500
}

void loop() {
    servo.run();                // call every loop iteration
}
```

## Minimal Example — Speed Control (DCMotorTacho)

```cpp
#include <Encoder.h>
#include <LMD18200.h>
#include <DCMotorServo.h>
#include <DCMotorTacho.h>

LMD18200 driver(6, 7, 8);
Encoder  enc(2, 3);

void motorWrite(int16_t s) { driver.write(s); }
void motorBrake()           { driver.brake(); }
long encRead()              { return enc.read(); }
void encWrite(long p)       { enc.write(p); }

DCMotorServo servo(motorWrite, motorBrake, encRead, encWrite);

#define CPR 912.0   // counts per revolution — see tuning.md for how to determine this
DCMotorTacho tacho(&servo, CPR);

void setup() {
    driver.begin();
    tacho.setPIDTunings(0.6, 3.0);
    tacho.setFeedforward(10, 24); // from a PWM sweep: see tuning.md
    tacho.setTrimLimit(20);
    tacho.setSpeedRPM(60);      // target 60 RPM
}

void loop() {
    tacho.run();                // runs the speed loop
}
```

---

## Next Steps

- See [api.md](api.md) for all methods and their parameters.
- See [tuning.md](tuning.md) for a step-by-step calibration procedure.
- Browse the `examples/` folder for driver-specific wiring and full sketches.

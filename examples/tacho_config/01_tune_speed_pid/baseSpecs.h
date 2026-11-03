// baseSpecs.h — hardware and calibration specs for the config sketches.
//
// Fill this out as you work through the configuration procedure and copy the
// file forward into the next step's sketch folder (each sketch folder carries
// its own copy; the Arduino IDE shows it as a tab). The values below are the
// reference-rig results (26PG-3429-19-EN gearmotor).

#ifndef BASESPECS_H
#define BASESPECS_H

#include <Encoder.h>

// ----- Driver selection (exactly one) -----
#define USE_LMD18500 1
// #define USE_L298N 1

// ----- LMD18200 pin definitions -----
#ifdef USE_LMD18500
#include "LMD18200.h"
#define LMD_PWM_PIN 6
#define LMD_DIR_PIN 7
#define LMD_BRAKE_PIN 8
#define ENCODER_PIN1 A2 // non-interrupt pins
#define ENCODER_PIN2 A3
#endif // USE_LMD18500

// ----- L298N pin definitions -----
#ifdef USE_L298N
#include "L298N.h"
#define L298_ENA 6
#define L298_IN1 7
#define L298_IN2 8
#define ENCODER_PIN1 2 // interrupt pins
#define ENCODER_PIN2 3
#endif // USE_L298N

// ----- Motor specs (datasheet) -----
#define PPR 12        // encoder pulses per motor-shaft revolution
#define GEAR_RATIO 19 // gearbox reduction
// Measured in servo_config/04 — start a new motor at 1.0
#define EMPIRICAL_FUDGE_FACTOR 0.875
#define ENCODER_RESOLUTION (PPR * GEAR_RATIO)
#define CPR (ENCODER_RESOLUTION * 4 * EMPIRICAL_FUDGE_FACTOR)

// ----- Determined by the config procedure -----
#define PWM_SKIP 35 // servo_config/01
#define ACCURACY 8  // servo_config/02
// Position-loop gains (servo_config/03)
#define SERVO_KP 0.1
#define SERVO_KI 0.15
#define SERVO_KD 0.05
// Speed loop (tacho_config)
#define SPEED_INTERVAL 20 // ms
#define SPEED_KP 0.6      // PWM per RPM
#define SPEED_KI 3.0      // PWM per RPM-second
// Feedforward, from the PWM sweep (tacho_config/01): counts/s = FF_KV * (pwm - FF_STATIC)
#define FF_STATIC 0 // PWM
#define FF_KV 0     // counts/s per PWM step; 0 = no feedforward
#define TRIM_LIMIT 255 // PWM; about 20 once the feedforward is set

// ----- Shared driver/encoder instances and wrapper functions -----
#ifdef USE_LMD18500
LMD18200 motorDriver(LMD_PWM_PIN, LMD_DIR_PIN, LMD_BRAKE_PIN);
#endif // USE_LMD18500

#ifdef USE_L298N
L298N motorDriver(L298_ENA, L298_IN1, L298_IN2);
#endif // USE_L298N

Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

void wrapMotorWrite(int16_t speed) { motorDriver.write(speed); }
void wrapMotorBrake() { motorDriver.brake(); }
long wrapEncoderRead() { return myEncoder.read(); }
void wrapEncoderWrite(long newPosition) { myEncoder.write(newPosition); }

#endif // BASESPECS_H

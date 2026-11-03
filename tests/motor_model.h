// A first-order DC motor for the speed-loop tests: above a deadband of
// DEADBAND PWM it turns at KV counts/s per PWM step, approached with time
// constant TAU_MS. step() advances it 1 ms, reading the drive from
// fake_motor_speed and writing fake_encoder. A held shaft doesn't turn.
#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include "test_support.h"

struct MotorModel
{
  static constexpr double KV = 24.3;
  static constexpr double DEADBAND = 10;
  static constexpr double TAU_MS = 30;

  double countsPerS = 0;
  double position = 0;
  bool held = false;

  void step()
  {
    double pwm = fake_motor_speed;
    double mag = (pwm < 0 ? -pwm : pwm) - DEADBAND;
    double target = (mag > 0) ? (pwm < 0 ? -1 : 1) * KV * mag : 0;
    countsPerS += (target - countsPerS) / TAU_MS;
    if (held)
      countsPerS = 0;
    position += countsPerS / 1000.0;
    fake_encoder = (long)position;
    fake_millis += 1;
  }
};

#endif

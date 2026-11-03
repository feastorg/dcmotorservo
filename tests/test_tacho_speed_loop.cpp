// #26: the speed loop drives PWM directly, with feedforward and a bounded,
// conditionally integrated correction. Against a first-order motor model:
// steady speed across the range, a held shaft that doesn't lunge on
// release, a bounded drive while held, and no backward drive on entry from
// reverse.
#include <cmath>
#include <vector>

#include <DCMotorTacho.h>

#include "motor_model.h"

static const double CPR = 848;

// Resets the fakes before Rig's members read them.
struct FreshFakes
{
  FreshFakes() { resetFakes(); }
};

struct Rig : FreshFakes
{
  DCMotorServo servo{fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite};
  DCMotorTacho tacho{&servo, CPR, 5};
  MotorModel motor;
  std::vector<long> pos; // encoder count at each ms
  std::vector<int16_t> pwm;

  Rig(bool feedforward)
  {
    servo.setMaxPWM(150);
    tacho.setPIDTunings(0.6, 3.0);
    if (feedforward)
    {
      tacho.setFeedforward(MotorModel::DEADBAND, MotorModel::KV);
      tacho.setTrimLimit(20);
    }
  }

  void runFor(unsigned long ms)
  {
    for (unsigned long i = 0; i < ms; i++)
    {
      tacho.run();
      motor.step();
      pos.push_back(fake_encoder);
      pwm.push_back(fake_motor_speed);
    }
  }

  // Mean RPM over [from, to) ms of the log.
  double rpm(size_t from, size_t to) const
  {
    return (double)(pos[to - 1] - pos[from]) / CPR * 60000.0 / (double)(to - 1 - from);
  }
};

int main()
{
  // Steady speed, with feedforward.
  for (double target : {10.0, 60.0, 150.0, -60.0})
  {
    Rig r(true);
    r.tacho.setSpeedRPM(target);
    r.runFor(20000);
    double lo = 1e9, hi = -1e9;
    for (size_t t = 10000; t + 1000 <= 20000; t += 100)
    {
      double v = r.rpm(t, t + 1000);
      lo = v < lo ? v : lo;
      hi = v > hi ? v : hi;
    }
    fprintf(stderr, "%.0f RPM with feedforward: 1 s means %.2f to %.2f from 10 s\n", target, lo, hi);
    CHECK(lo > target - 0.5 && hi < target + 0.5);
  }

  // With the trim limited to 5 the feedforward has to carry the speed, in
  // either direction.
  for (double target : {60.0, -60.0})
  {
    Rig r(true);
    r.tacho.setTrimLimit(5);
    r.tacho.setSpeedRPM(target);
    r.runFor(10000);
    double v = r.rpm(5000, 10000);
    fprintf(stderr, "%.0f RPM with the trim limited to 5: %.2f RPM over 5-10 s\n", target, v);
    CHECK(std::fabs(v - target) < 0.5);
  }

  // Without feedforward the PI alone gets there.
  {
    Rig r(false);
    r.tacho.setSpeedRPM(60);
    r.runFor(20000);
    double v = r.rpm(15000, 20000);
    fprintf(stderr, "60 RPM without feedforward: %.2f RPM over 15-20 s\n", v);
    CHECK(std::fabs(v - 60) < 1.0);
  }

  // Held for 0.5 s at 120 RPM, then released: no lunge.
  {
    Rig r(true);
    r.tacho.setSpeedRPM(120);
    r.runFor(5000);
    r.motor.held = true;
    r.runFor(500);
    r.motor.held = false;
    r.runFor(5000);
    double peak = 0;
    for (size_t t = 5500; t + 1000 <= 10500; t += 50)
    {
      double v = r.rpm(t, t + 1000);
      peak = v > peak ? v : peak;
    }
    fprintf(stderr, "120 RPM, released after 0.5 s held: peak 1 s mean %.2f RPM\n", peak);
    CHECK(peak < 125);
  }

  // Held for 2 s: the drive stays within the feedforward plus the trim limit.
  {
    Rig r(true);
    r.tacho.setSpeedRPM(60);
    r.runFor(3000);
    r.motor.held = true;
    r.runFor(2000);
    double ff = MotorModel::DEADBAND + 60 * CPR / 60.0 / MotorModel::KV;
    int16_t most = 0;
    for (size_t t = 3000; t < 5000; t++)
      most = r.pwm[t] > most ? r.pwm[t] : most;
    fprintf(stderr, "60 RPM held 2 s: highest PWM %d, feedforward %.1f + trim 20\n", most, ff);
    CHECK(most <= (int16_t)(ff + 20 + 1));
  }

  // Entering at +60 RPM with the shaft turning at -150: never driven backwards.
  for (bool feedforward : {true, false})
  {
    Rig r(feedforward);
    fake_motor_speed = -150;
    for (int i = 0; i < 1000; i++)
      r.motor.step();
    r.tacho.setSpeedRPM(60);
    r.runFor(1000);
    int16_t lowest = 0;
    for (int16_t p : r.pwm)
      lowest = p < lowest ? p : lowest;
    fprintf(stderr, "entry at +60 from -150 RPM (%s feedforward): lowest PWM %d\n", feedforward ? "with" : "without",
            lowest);
    CHECK(lowest >= 0);
  }

  // Setpoint 0 brakes and resets: a restart drives as a fresh start does.
  // Without feedforward the integral carries the drive, so there is one to
  // reset.
  {
    Rig fresh(false);
    fresh.tacho.setSpeedRPM(60);
    fresh.runFor(200);

    Rig again(false);
    again.tacho.setSpeedRPM(60);
    again.runFor(3000);
    CHECK(fake_motor_speed > 30); // the integral carries this
    again.tacho.setSpeedRPM(0);
    again.tacho.run();
    CHECK(fake_motor_speed == 0);
    again.motor.countsPerS = 0;
    again.motor.position = (double)fake_encoder; // no fractional count carried over
    again.pos.clear();
    again.pwm.clear();
    again.tacho.setSpeedRPM(60);
    again.runFor(200);
    bool same = true;
    for (size_t i = 0; i < 200; i++)
      same = same && (again.pwm[i] == fresh.pwm[i]);
    fprintf(stderr, "restart after stop drives as a fresh start: %s\n", same ? "yes" : "no");
    CHECK(same);
  }

  return test_failures ? 1 : 0;
}

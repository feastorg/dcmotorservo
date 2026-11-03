// #26: inside the position deadband run() keeps the PID's integral instead of
// resetting it, and run() never drives away from the target.
#include <DCMotorServo.h>

#include "test_support.h"

static DCMotorServo *makeServo()
{
  DCMotorServo *servo = new DCMotorServo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo->setPWMSkip(15);
  servo->setMaxPWM(150);
  servo->setAccuracy(10);
  servo->setPIDTunings(0.01, 0.5, 0); // mostly integral, so the integral shows
  return servo;
}

static void runFor(DCMotorServo *servo, unsigned long ms)
{
  for (unsigned long i = 0; i < ms; i++)
  {
    fake_millis += 1;
    servo->run();
  }
}

int main()
{
  // Build an integral with the shaft held 100 counts short, then put the
  // shaft inside the deadband, then 20 counts short: the drive picks up
  // from the integral built before.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    servo->moveTo(100);
    runFor(servo, 1000);
    int16_t built = fake_motor_speed;

    fake_encoder = 95;
    runFor(servo, 500);
    CHECK(fake_motor_speed == 0);

    fake_encoder = 80;
    runFor(servo, 60);
    fprintf(stderr, "deadband: drive %d after building, %d after the deadband\n", built, fake_motor_speed);
    CHECK(built > 15);
    CHECK(fake_motor_speed > 15 + 10); // a reset integral would give about pwm_skip
  }

  // Build a negative integral with the shaft held above the target, then
  // put it just below: the error is positive, the PID output still
  // negative. No drive, rather than a kick away from the target.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    fake_encoder = 200;
    servo->moveTo(100);
    runFor(servo, 1000);
    CHECK(fake_motor_speed < 0);

    fake_encoder = 85;
    bool away = false;
    for (int i = 0; i < 200; i++)
    {
      fake_millis += 1;
      servo->run();
      away = away || fake_motor_speed < 0;
    }
    fprintf(stderr, "no drive away: shaft 15 below the target, drive %d, ever negative: %s\n", fake_motor_speed,
            away ? "yes" : "no");
    CHECK(!away);
  }

  return test_failures ? 1 : 0;
}

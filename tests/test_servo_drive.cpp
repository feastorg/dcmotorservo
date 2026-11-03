// #26: drive() writes the PWM directly, with run()'s protections: the target
// follows the shaft, a stall latches, and an endstop or travel limit in the
// direction of drive brakes.
#include <DCMotorServo.h>

#include "test_support.h"

static bool minTriggered = false;
static bool minStop() { return minTriggered; }

static DCMotorServo *makeServo()
{
  DCMotorServo *servo = new DCMotorServo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo->setMaxPWM(150);
  servo->setAccuracy(10);
  return servo;
}

int main()
{
  // Writes the PWM, clamped to maxPWM, and the target follows the shaft.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    servo->drive(80);
    CHECK(fake_motor_speed == 80);
    servo->drive(-200);
    CHECK(fake_motor_speed == -150);
    fake_encoder = 1234;
    servo->drive(40);
    CHECK(servo->getRequestedPosition() == 1234);
  }

  // A stall latches and brakes, and drive() then does nothing.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    servo->enableStallDetection(1000, 4);
    for (int i = 0; i < 1100; i++)
    {
      fake_millis += 1;
      servo->drive(60);
    }
    CHECK(servo->isStalled());
    CHECK(fake_motor_speed == 0);
    servo->drive(60);
    CHECK(fake_motor_speed == 0);
  }

  // A triggered endstop brakes drive towards it, not away from it.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    servo->attachEndstops(minStop, nullptr);
    minTriggered = true;
    servo->drive(-60);
    CHECK(fake_motor_speed == 0);
    servo->drive(60);
    CHECK(fake_motor_speed == 60);
    minTriggered = false;
  }

  // A travel limit brakes drive past it, not back from it.
  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    servo->setTravelLimits(-100, 100);
    fake_encoder = 100;
    servo->drive(60);
    CHECK(fake_motor_speed == 0);
    servo->drive(-60);
    CHECK(fake_motor_speed == -60);
  }

  return test_failures ? 1 : 0;
}

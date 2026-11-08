// A long position move with the drive saturated must not wind up the PID's
// integral: when the shaft coasts past the target the wound integral points
// away from it, run() refuses to drive away, and the motor sits outside the
// deadband with no drive until the integral unwinds (minutes at a low Ki).
#include <DCMotorServo.h>

#include "motor_model.h"
#include "test_support.h"

static void longMove(long from, long to)
{
  resetFakes();
  MotorModel motor;
  motor.position = from;
  fake_encoder = from;
  DCMotorServo *servo = new DCMotorServo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo->setPWMSkip(15);
  servo->setMaxPWM(150);
  servo->setAccuracy(10);
  servo->setPIDTunings(0.15, 0.01, 0.01);
  servo->moveTo(to);

  double overshoot = 0;
  for (int ms = 0; ms < 10000; ms++)
  {
    servo->run();
    motor.step();
    double past = (to > from) ? motor.position - to : to - motor.position;
    if (past > overshoot)
      overshoot = past;
  }
  long err = fake_encoder - to;
  fprintf(stderr, "move %ld -> %ld: ends at %ld (error %ld), drive %d, peak overshoot %.0f\n", from, to, fake_encoder,
          err, fake_motor_speed, overshoot);
  CHECK(err > -10 && err < 10);
  CHECK(overshoot < 30);
  delete servo;
}

int main()
{
  longMove(0, 2000);
  longMove(-7282, 0);
  longMove(-16622, 0);
  longMove(0, 16000);
  return test_failures ? 1 : 0;
}

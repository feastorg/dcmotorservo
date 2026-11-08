// DCMotorServo.cpp
#include "DCMotorServo.h"
#include <Arduino.h>

DCMotorServo::DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite)
    : _maxPWM(255), motorWrite(mWrite), motorBrake(mBrake), encoderRead(eRead), encoderWrite(eWrite)
{
  _PWM_output = 0;
  _pwm_skip = 50;
  _position_accuracy = 30;

  _limits_enabled = false;
  _limit_min = _limit_max = 0;
  endstopMin = nullptr;
  endstopMax = nullptr;
  _stall_enabled = false;
  _stalled = false;
  _driving = false;
  _stall_timeout = 0;
  _stall_min_counts = 4;
  _stall_last_time = 0;
  _stall_last_pos = 0;
  _homing = false;
  _homed = false;
  _homing_dir = 0;
  _homing_pwm = 0;
  _homing_start_pos = 0;
  _homing_max_travel = 0;

  _PID_input = encoderRead();
  _PID_output = 0;
  _PID_setpoint = _PID_input;

  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint, 0.1, 0.2, 0.1, DIRECT);

  // Configure PID controller
  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip - 255, 255 - _pwm_skip);
  myPID->SetMode(AUTOMATIC);
}

void DCMotorServo::setCurrentPosition(long new_position)
{
  encoderWrite(new_position);
  _PID_input = encoderRead();
}

bool DCMotorServo::setPWMSkip(uint8_t range)
{
  if (range < 255)
  {
    _pwm_skip = range;
    return true;
  }
  return false;
}

void DCMotorServo::setAccuracy(unsigned int range)
{
  _position_accuracy = range;
}

void DCMotorServo::setMaxPWM(uint8_t maxPWM)
{
  _maxPWM = maxPWM;
  myPID->SetOutputLimits(_pwm_skip - _maxPWM, _maxPWM - _pwm_skip); // pwm_skip is added to the output in run()
}

void DCMotorServo::setPIDTunings(double Kp, double Ki, double Kd)
{
  myPID->SetTunings(Kp, Ki, Kd);
}

bool DCMotorServo::finished()
{
  return !_homing && fabs(_PID_setpoint - _PID_input) < _position_accuracy && _PWM_output == 0;
}

void DCMotorServo::move(long new_rela_position)
{
  moveTo((long)_PID_setpoint + new_rela_position);
}

void DCMotorServo::moveTo(long new_position)
{
  _PID_setpoint = clampToLimits(new_position);
}

void DCMotorServo::setTravelLimits(long min_position, long max_position)
{
  if (min_position > max_position)
  {
    long tmp = min_position;
    min_position = max_position;
    max_position = tmp;
  }
  _limit_min = min_position;
  _limit_max = max_position;
  _limits_enabled = true;
  _PID_setpoint = clampToLimits((long)_PID_setpoint);
}

void DCMotorServo::clearTravelLimits()
{
  _limits_enabled = false;
}

long DCMotorServo::clampToLimits(long position)
{
  if (!_limits_enabled)
    return position;
  if (position < _limit_min)
    return _limit_min;
  if (position > _limit_max)
    return _limit_max;
  return position;
}

void DCMotorServo::attachEndstops(EndstopReadFunc minStop, EndstopReadFunc maxStop)
{
  endstopMin = minStop;
  endstopMax = maxStop;
}

void DCMotorServo::enableStallDetection(unsigned long timeout_ms, long min_counts)
{
  _stall_timeout = timeout_ms;
  _stall_min_counts = min_counts;
  _stall_enabled = true;
  resetStallWindow();
}

void DCMotorServo::disableStallDetection()
{
  _stall_enabled = false;
}

bool DCMotorServo::isStalled()
{
  return _stalled;
}

void DCMotorServo::clearStall()
{
  _stalled = false;
  resetStallWindow();
}

void DCMotorServo::resetStallWindow()
{
  _stall_last_pos = encoderRead();
  _stall_last_time = millis();
}

bool DCMotorServo::stallDetected()
{
  long pos = encoderRead();
  unsigned long now = millis();
  if (labs(pos - _stall_last_pos) >= _stall_min_counts)
  {
    _stall_last_pos = pos;
    _stall_last_time = now;
    return false;
  }
  return (now - _stall_last_time) >= _stall_timeout;
}

bool DCMotorServo::startHoming(int8_t direction, uint8_t pwm, long max_travel)
{
  if (direction == 0)
    return false;
  EndstopReadFunc target = (direction < 0) ? endstopMin : endstopMax;
  if (target == nullptr && !_stall_enabled)
    return false; // nothing can detect the extreme
  uint8_t lowest = (_pwm_skip < _maxPWM) ? _pwm_skip : _maxPWM; // guard inverted config
  _homing_pwm = constrain(pwm, lowest, _maxPWM);
  _homing_dir = (direction < 0) ? -1 : 1;
  _homing_max_travel = (max_travel < 0) ? -max_travel : max_travel;
  _homing_start_pos = encoderRead();
  _homed = false;
  _homing = true;
  _stalled = false;
  _driving = false;
  myPID->SetMode(MANUAL);
  return true;
}

bool DCMotorServo::isHoming()
{
  return _homing;
}

bool DCMotorServo::isHomed()
{
  return _homed;
}

void DCMotorServo::runHoming()
{
  EndstopReadFunc target = (_homing_dir < 0) ? endstopMin : endstopMax;

  // Termination conditions can vanish mid-move (stall detection disabled,
  // endstops detached): abort rather than drive into the hard stop forever.
  if (target == nullptr && !_stall_enabled)
  {
    finishHoming(false);
    return;
  }

  bool reached = (target != nullptr && target());
  // Sensorless homing: a stall marks the extreme. Only valid once the motor
  // has actually been driven, so the window can't expire during idle time
  // between startHoming() and the first run().
  if (!reached && _stall_enabled && _driving)
    reached = stallDetected();
  if (reached)
  {
    finishHoming(true);
    return;
  }

  // Failsafe against a dead endstop switch: bounded travel aborts unhomed
  if (_homing_max_travel > 0 && labs(encoderRead() - _homing_start_pos) >= _homing_max_travel)
  {
    finishHoming(false);
    return;
  }

  if (!_driving)
    resetStallWindow(); // start the no-movement window with the first driven cycle
  _driving = true;
  _PWM_output = _homing_pwm;
  motorWrite(_homing_dir < 0 ? -(int16_t)_homing_pwm : (int16_t)_homing_pwm);
}

void DCMotorServo::finishHoming(bool success)
{
  haltMotor();
  _homing = false;
  _homed = success;
  if (success)
  {
    encoderWrite(0);
    _PID_input = 0;
    _PID_setpoint = clampToLimits(0);
  }
  else
  {
    // Hold where we are; don't chase the pre-homing target
    _PID_input = encoderRead();
    _PID_setpoint = _PID_input;
  }
}

void DCMotorServo::haltMotor()
{
  myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  _driving = false;
  motorBrake();
}

long DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

long DCMotorServo::getActualPosition()
{
  return encoderRead();
}

void DCMotorServo::run()
{
  if (_homing)
  {
    runHoming();
    return;
  }
  if (_stalled)
    return; // latched fault: motor stays braked until clearStall()

  _PID_input = encoderRead();
  double error = _PID_setpoint - _PID_input;

  // Hold braked while the target lies beyond a triggered endstop. Judged on
  // the error direction (not the PID output, which is zeroed by the halt) so
  // the hold is stable across cycles; motion away from the endstop is allowed.
  if ((error < 0 && endstopMin != nullptr && endstopMin()) ||
      (error > 0 && endstopMax != nullptr && endstopMax()))
  {
    // Pull the unreachable target to the held position. Otherwise a caller
    // (or DCMotorTacho's speed loop) could keep winding the setpoint past the
    // stop, storing up an uncommanded lunge for the moment the switch releases.
    _PID_setpoint = _PID_input;
    haltMotor();
    return;
  }

  // Inside the deadband: no drive, and the PID is not computed, so its
  // integral is held for the next move instead of being reset (#26).
  int finalSpeed = 0;
  int outputPWM = 0;
  if (fabs(error) < _position_accuracy)
  {
    _PID_output = 0; // PID_v1 keeps its integral internally; this is the reported output
  }
  else
  {
    myPID->SetMode(AUTOMATIC); // re-arm after stop(); no-op when already AUTOMATIC
    myPID->Compute();
    // No integral while the drive is saturated. A long move otherwise winds
    // it up; when the shaft coasts past the target the integral points away
    // from it, the rule below zeroes the drive, and the motor sits outside
    // the deadband until the integral unwinds. PID_v1 doesn't expose its
    // integral: MANUAL -> AUTOMATIC sets it to the output, here 0.
    if (fabs(_PID_output) + _pwm_skip >= _maxPWM)
    {
      double out = _PID_output;
      myPID->SetMode(MANUAL);
      _PID_output = 0;
      myPID->SetMode(AUTOMATIC);
      _PID_output = out;
    }
    // A PWM value that adds a base skip value.
    outputPWM = constrain(abs(_PID_output) + _pwm_skip, _pwm_skip, _maxPWM);
    // Zero output means no drive: never map it to +pwm_skip.
    finalSpeed = (_PID_output == 0) ? 0 : ((_PID_output < 0) ? -outputPWM : outputPWM);
    // Never drive away from the target. Near zero error a small output of
    // the wrong sign (the integral, or the derivative as the shaft reverses)
    // would otherwise become a full pwm_skip kick backwards.
    if ((error > 0 && finalSpeed < 0) || (error < 0 && finalSpeed > 0))
      finalSpeed = 0;
  }
  _PWM_output = (finalSpeed == 0) ? 0 : outputPWM;

  // Send the computed speed via the user-supplied function
  motorWrite(finalSpeed);

  watchStall(finalSpeed);
}

void DCMotorServo::watchStall(int16_t speed)
{
  // Stall fault: motor driven but encoder not advancing
  if (_stall_enabled && speed != 0)
  {
    if (!_driving)
      resetStallWindow(); // idle -> driving transition: the window starts now
    _driving = true;
    if (stallDetected())
    {
      _stalled = true;
      haltMotor();
    }
  }
  else
  {
    _driving = false;
  }
}

void DCMotorServo::drive(int16_t pwm)
{
  if (_homing || _stalled)
    return; // run() owns homing; a latched stall stays braked
  // Hold the target at the shaft, so a later run() holds here.
  _PID_input = encoderRead();
  _PID_setpoint = _PID_input;
  myPID->SetMode(MANUAL);
  _PID_output = 0;

  // The same protections as position mode: a triggered endstop, or a
  // software travel limit, in the direction of drive brakes the motor.
  long pos = (long)_PID_input;
  if ((pwm < 0 && ((endstopMin != nullptr && endstopMin()) || (_limits_enabled && pos <= _limit_min))) ||
      (pwm > 0 && ((endstopMax != nullptr && endstopMax()) || (_limits_enabled && pos >= _limit_max))))
  {
    _PID_setpoint = clampToLimits(pos);
    haltMotor();
    return;
  }

  pwm = constrain(pwm, -(int16_t)_maxPWM, (int16_t)_maxPWM);
  _PWM_output = (uint8_t)((pwm < 0) ? -pwm : pwm);
  motorWrite(pwm);
  watchStall(pwm);
}

uint8_t DCMotorServo::getMaxPWM()
{
  return _maxPWM;
}

void DCMotorServo::stop()
{
  // Sync the target to the current position so the next run() holds here
  // instead of chasing a stale setpoint (pre-homing, or wound up by
  // DCMotorTacho's speed loop).
  _homing = false;
  _PID_input = encoderRead();
  _PID_setpoint = _PID_input;
  haltMotor();
}

String DCMotorServo::getDebugInfo()
{
  String info = "DCMotorServo Debug Info:\n";
  info += "Encoder Position: " + String(encoderRead()) + "\n";
  info += "PID Setpoint: " + String(_PID_setpoint) + "\n";
  info += "PID Input: " + String(_PID_input) + "\n";
  info += "PID Output: " + String(_PID_output) + "\n";
  info += "PWM Skip: " + String(_pwm_skip) + "\n";
  info += "Position Accuracy: " + String(_position_accuracy) + "\n";
  if (_limits_enabled)
    info += "Travel Limits: [" + String(_limit_min) + ", " + String(_limit_max) + "]\n";
  info += "Stalled: " + String(_stalled ? "yes" : "no") + ", Homing: " + String(_homing ? "yes" : "no") + ", Homed: " + String(_homed ? "yes" : "no") + "\n";

  double Kp = myPID->GetKp();
  double Ki = myPID->GetKi();
  double Kd = myPID->GetKd();
  info += "PID Params - Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd: " + String(Kd) + "\n";

  return info;
}

String DCMotorServo::getSerialPlotter(int id)
{
  String suffix = (id >= 0) ? String(id) : "";
  String data = "Setpoint" + suffix + ":" + String(_PID_setpoint, 2) +
                ", Input" + suffix + ":" + String(_PID_input, 2) +
                ", Output" + suffix + ":" + String(_PID_output, 2);
  return data;
}

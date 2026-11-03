#include "DCMotorTacho.h"
#include <Arduino.h>

DCMotorTacho::DCMotorTacho(DCMotorServo *servo, double cpr, unsigned long speedInterval)
    : _servo(servo), _CPR(cpr), _speedInterval(speedInterval)
{
    _desiredSpeedRPM = 0;
    _measuredSpeedRPM = 0.0;
    _kp = 0.6;
    _ki = 3.0;
    _ffStatic = 0;
    _ffKv = 0;
    _trimLimit = 255;
    _integral = 0;
    _drivePWM = 0;
    // Samples spanning about 50 ms, at least 1 and at most WINDOW_MAX.
    unsigned long span = (_speedInterval > 0) ? (50 + _speedInterval - 1) / _speedInterval : WINDOW_MAX;
    _winSpan = (span < 1) ? 1 : (span > WINDOW_MAX ? WINDOW_MAX : (uint8_t)span);
    restartMeasurement();
}

void DCMotorTacho::setSpeedRPM(double rpm)
{
    if (_desiredSpeedRPM == 0 && rpm != 0)
    {
        // Starting: measure from here, not from samples that may predate
        // position or open-loop travel.
        restartMeasurement();
        holdSpeedLoop();
    }
    _desiredSpeedRPM = rpm;
    if (rpm == 0)
    {
        holdSpeedLoop();
        _servo->stop();
    }
}

double DCMotorTacho::getDesiredSpeedRPM() const
{
    return _desiredSpeedRPM;
}

void DCMotorTacho::setPIDTunings(double Kp, double Ki)
{
    _kp = Kp;
    _ki = Ki;
}

void DCMotorTacho::setFeedforward(double pwmStatic, double kv)
{
    _ffStatic = (pwmStatic < 0) ? 0 : pwmStatic;
    _ffKv = (kv < 0) ? 0 : kv;
}

void DCMotorTacho::setTrimLimit(double pwm)
{
    _trimLimit = (pwm < 0) ? -pwm : pwm;
    if (_integral > _trimLimit)
        _integral = _trimLimit;
    else if (_integral < -_trimLimit)
        _integral = -_trimLimit;
}

double DCMotorTacho::getMeasuredSpeedRPM() const
{
    return _measuredSpeedRPM;
}

void DCMotorTacho::run()
{
    // Hold the speed loop while the servo is homing or stall-latched: the
    // servo's run() drives the homing move or holds the latched brake.
    if (_servo->isHoming() || _servo->isStalled())
    {
        holdSpeedLoop();
        _servo->run();
        // Restart AFTER run(): homing completion zeroes the encoder inside
        // it, and an earlier count would fake a huge speed sample.
        restartMeasurement();
        return;
    }

    if (_desiredSpeedRPM == 0)
    {
        holdSpeedLoop();
        _servo->stop();
        // Keep measuring, so the reading follows the shaft to rest, or a
        // shaft turned by hand.
        measureSpeed();
        return;
    }

    unsigned long before = _winTime[_winLen - 1];
    if (measureSpeed())
        updateDrive(_winTime[_winLen - 1] - before);
    _servo->drive(_drivePWM);
}

void DCMotorTacho::updateDrive(unsigned long dt)
{
    double maxPWM = _servo->getMaxPWM();
    double e = _desiredSpeedRPM - _measuredSpeedRPM;
    double ff = ((_desiredSpeedRPM > 0) ? _ffStatic : -_ffStatic);
    if (_ffKv > 0)
        ff += _desiredSpeedRPM * _CPR / 60.0 / _ffKv;

    // Conditional integration: not while the correction or the output is
    // at its limit in the error's direction.
    double trim = _kp * e + _integral;
    bool atLimit = (e > 0 && (trim >= _trimLimit || ff + trim >= maxPWM)) ||
                   (e < 0 && (trim <= -_trimLimit || ff + trim <= -maxPWM));
    if (!atLimit)
    {
        _integral += _ki * e * (double)dt / 1000.0;
        if (_integral > _trimLimit)
            _integral = _trimLimit;
        else if (_integral < -_trimLimit)
            _integral = -_trimLimit;
    }

    trim = constrain(_kp * e + _integral, -_trimLimit, _trimLimit);
    double u = constrain(ff + trim, -maxPWM, maxPWM);
    _drivePWM = (int16_t)((u >= 0) ? u + 0.5 : u - 0.5);
}

void DCMotorTacho::stop()
{
    _desiredSpeedRPM = 0;
    _servo->stop();
    holdSpeedLoop();
}

// Takes a speed sample once per speed interval. True when it took one.
bool DCMotorTacho::measureSpeed()
{
    unsigned long now = millis();
    if (now - _winTime[_winLen - 1] < _speedInterval)
        return false;
    // Keep at most _winSpan samples back, so the speed spans that many.
    if (_winLen > _winSpan)
    {
        for (uint8_t i = 1; i < _winLen; i++)
        {
            _winCount[i - 1] = _winCount[i];
            _winTime[i - 1] = _winTime[i];
        }
        _winLen--;
    }
    long count = _servo->getActualPosition();
    _measuredSpeedRPM = ((double)(count - _winCount[0]) / _CPR) * (60000.0 / (double)(now - _winTime[0]));
    _winCount[_winLen] = count;
    _winTime[_winLen] = now;
    _winLen++;
    return true;
}

void DCMotorTacho::restartMeasurement()
{
    _winCount[0] = _servo->getActualPosition();
    _winTime[0] = millis();
    _winLen = 1;
}

void DCMotorTacho::holdSpeedLoop()
{
    _integral = 0;
    _drivePWM = 0;
}

DCMotorServo *DCMotorTacho::getServo() const
{
    return _servo;
}

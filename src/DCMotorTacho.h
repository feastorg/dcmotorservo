#ifndef DCMotorTacho_h
#define DCMotorTacho_h

#include "DCMotorServo.h"
#include <Arduino.h>

/**
 * @class DCMotorTacho
 * @brief Closed-loop speed control for a DCMotorServo's motor.
 *
 * Every speed interval it measures the shaft's speed and drives the motor
 * through DCMotorServo::drive() with
 *
 *     pwm = feedforward + trim
 *     feedforward = sign(target) * pwmStatic + target_counts_per_s / kv
 *     trim = Kp * error + integral, bounded to +-trimLimit
 *
 * where error is the target RPM less the measured RPM. The integral does not
 * accumulate while the trim or the output is at its limit in the error's
 * direction (conditional integration), so a held shaft banks at most
 * trimLimit of drive. With no feedforward set (the default) it is a plain PI
 * on PWM.
 *
 * The speed is measured over the last ~50 ms of samples (at most 10), so
 * low speeds resolve to more than a count or two.
 *
 * A setpoint of 0 stops the motor (brake) and resets the integral. While the
 * servo is homing or stall-latched the speed loop is held and the servo's
 * run() progresses the homing move or holds the brake.
 */
class DCMotorTacho
{
public:
    /**
     * Constructor for DCMotorTacho.
     * @param servo Pointer to an existing DCMotorServo object.
     * @param cpr Counts per revolution (CPR) for the motor.
     * @param speedInterval Speed update interval in ms (default 50).
     */
    DCMotorTacho(DCMotorServo *servo, double cpr, unsigned long speedInterval = 50);

    /// Set the desired speed in RPM. 0 stops the motor.
    void setSpeedRPM(double rpm);
    /// Get the desired speed (RPM).
    double getDesiredSpeedRPM() const;
    /**
     * Set the speed loop's gains.
     * @param Kp PWM per RPM of speed error (default 0.6).
     * @param Ki PWM per RPM-second of speed error (default 3.0).
     */
    void setPIDTunings(double Kp, double Ki);
    /**
     * Set the feedforward: the drive the motor needs to turn at the target
     * speed with no correction. Both come from an open-loop PWM sweep: at
     * each PWM, the steady speed in counts/s; fit a line, counts/s =
     * kv * (pwm - pwmStatic). Set them low rather than high: feedforward
     * that under-drives costs a little start-up time, while over-drive
     * overshoots at start-up.
     * @param pwmStatic PWM that just starts the shaft turning (>= 0).
     * @param kv Counts/s per PWM step above pwmStatic; 0 (the default)
     *        leaves only the static term.
     */
    void setFeedforward(double pwmStatic, double kv);
    /**
     * Bound the correction (P + I) around the feedforward, in PWM. With
     * feedforward set, about 20 keeps a held shaft from banking more than
     * that and lunging on release. Default 255: no bound beyond maxPWM.
     */
    void setTrimLimit(double pwm);
    /**
     * Get the most recently measured speed (RPM). It is measured while the
     * setpoint is 0 as well, but not while the servo is homing or
     * stall-latched.
     */
    double getMeasuredSpeedRPM() const;
    /**
     * Run the speed loop. Call every loop iteration. If the desired speed is
     * 0 the motor is stopped. While the servo is homing or stall-latched
     * only the servo's run() is called; speed control resumes afterwards.
     */
    void run();

    /// Stop the motor and reset the speed loop.
    void stop();
    /// Get a pointer to the underlying DCMotorServo.
    DCMotorServo *getServo() const;

private:
    static const uint8_t WINDOW_MAX = 10; // speed samples in the measurement window

    DCMotorServo *_servo;         // Underlying motor.
    double _desiredSpeedRPM;      // Desired speed (RPM) command.
    double _measuredSpeedRPM;     // Measured speed (RPM).
    double _CPR;                  // Encoder counts per revolution.
    unsigned long _speedInterval; // Speed update interval in ms.

    double _kp, _ki;       // Speed loop gains (PWM per RPM, per RPM-s).
    double _ffStatic;      // Feedforward static PWM.
    double _ffKv;          // Feedforward counts/s per PWM step; 0 = no speed term.
    double _trimLimit;     // Bound on the correction around the feedforward, PWM.
    double _integral;      // Speed loop integral, PWM.
    int16_t _drivePWM;     // PWM from the last speed sample.

    // Speed measurement: the encoder count and time at the last samples,
    // oldest first. The newest is the last sample's.
    long _winCount[WINDOW_MAX + 1];
    unsigned long _winTime[WINDOW_MAX + 1];
    uint8_t _winLen;  // entries in use
    uint8_t _winSpan; // samples the window spans

    void holdSpeedLoop();      // Reset the integral and the drive.
    void restartMeasurement(); // Measure from the shaft's position now.
    bool measureSpeed();       // Take a speed sample if an interval has passed.
    void updateDrive(unsigned long dt); // Compute _drivePWM from a new sample.
};

#endif // DCMotorTacho_h

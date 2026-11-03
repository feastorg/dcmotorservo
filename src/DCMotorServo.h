// DCMotorServo.h
#ifndef DCMotorServo_h
#define DCMotorServo_h

#include <PID_v1.h>
#include <Arduino.h>

// Define function pointer types for hardware abstraction:
typedef void (*MotorWriteFunc)(int16_t speed);
typedef void (*MotorBrakeFunc)();
typedef long (*EncoderReadFunc)();
typedef void (*EncoderWriteFunc)(long newPosition);
typedef bool (*EndstopReadFunc)(); // true = endstop triggered

/**
 * @class DCMotorServo
 * @brief A class to control DC motors with encoder feedback using PID.
 *
 * Provides methods to control a DC motor's position or speed using a PID controller and feedback from an encoder.
 * This version is driver-agnostic: hardware-specific operations are provided via function pointers.
 */
class DCMotorServo
{
public:
  /**
   * Constructor for the DCMotorServo class.
   * @param mWrite Function pointer for writing a motor command (signed speed value).
   * @param mBrake Function pointer for braking the motor.
   * @param eRead  Function pointer for reading the encoder position.
   * @param eWrite Function pointer for writing/resetting the encoder position.
   */
  DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite);

  PID *myPID; ///< Pointer to PID object for motor control.

  /**
   * Runs the PID controller and sends the computed output to the motor driver.
   * Special modes take precedence over the PID: while homing this drives the
   * homing move; while a stall fault is latched it does nothing (motor stays
   * braked); while the target lies beyond a triggered endstop it holds the
   * motor braked and pulls the target to the held position.
   */
  void run();

  /**
   * Stops the motor by invoking the brake function and syncs the target to
   * the current position, so the motor holds where it stopped instead of
   * chasing the old setpoint. Also cancels an in-progress homing move.
   */
  void stop();

  /**
   * Drives the motor at a signed PWM directly, bypassing the position PID.
   * For a speed loop such as DCMotorTacho; call it every pass in place of
   * run(). The target follows the shaft, so a later run() holds where the
   * motor is. Stall detection, attached endstops and software travel limits
   * apply as in run(): a stall latches, and an endstop or limit in the
   * direction of drive brakes the motor. Does nothing while homing or while
   * a stall is latched.
   * @param pwm Signed PWM, clamped to +-maxPWM. 0 coasts at 0 drive.
   */
  void drive(int16_t pwm);

  /**
   * Moves the motor to a new relative position.
   * @param new_rela_position The relative position to add to the current target.
   */
  void move(long new_rela_position);

  /**
   * Moves the motor to an absolute position.
   * @param new_position The target position.
   */
  void moveTo(long new_position);

  /**
   * Retrieves the current target position.
   * @return The motor's target position.
   */
  long getRequestedPosition();

  /**
   * Retrieves the actual current position from the encoder.
   * @return The current encoder reading.
   */
  long getActualPosition();

  /**
   * Checks if the motor has reached its target position within defined accuracy.
   * Always false while a homing move is in progress. A target blocked by a
   * triggered endstop is pulled to the held position, so this reports true
   * once the motor has gone as far as it can.
   * @return True if the motor is at the target position, false otherwise.
   */
  bool finished();

  /**
   * Sets the PWM skip value, which defines the minimal PWM level to overcome low-power stalling.
   * @param range Minimum PWM value.
   * @return True if set successfully, false otherwise.
   */
  bool setPWMSkip(uint8_t range);

  /**
   * Sets the maximum PWM value allowed.
   * @param maxPWM Maximum PWM value.
   */
  void setMaxPWM(uint8_t maxPWM);

  /**
   * Gets the maximum PWM value allowed.
   * @return Maximum PWM value.
   */
  uint8_t getMaxPWM();

  /**
   * Sets the PID tuning parameters.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void setPIDTunings(double Kp, double Ki, double Kd);

  /**
   * Sets the tolerance for position accuracy.
   * @param range Maximum allowable error.
   */
  void setAccuracy(unsigned int range);

  /**
   * Sets the current encoder position.
   * @param new_position The new encoder position.
   */
  void setCurrentPosition(long new_position);

  /**
   * Sets software travel limits (extrema in encoder counts).
   * Targets passed to move()/moveTo() are clamped into [min_position, max_position].
   * @param min_position Lowest allowed target position.
   * @param max_position Highest allowed target position.
   */
  void setTravelLimits(long min_position, long max_position);

  /**
   * Removes software travel limits.
   */
  void clearTravelLimits();

  /**
   * Attaches physical endstop sensors (limit switch, hall effect, ...).
   * While a triggered endstop blocks the direction the target lies in, run()
   * holds the motor braked and pulls the target to the held position (so a
   * later switch release cannot cause an uncommanded move); motion away from
   * the endstop remains allowed.
   * @param minStop Function returning true when the minimum-side endstop is triggered (or nullptr).
   * @param maxStop Function returning true when the maximum-side endstop is triggered (or nullptr).
   */
  void attachEndstops(EndstopReadFunc minStop, EndstopReadFunc maxStop);

  /**
   * Enables encoder-based stall detection: if the motor is driven but the encoder
   * advances less than min_counts within timeout_ms, the motor is braked and a
   * stall fault is latched (see isStalled()/clearStall()). During homing a stall
   * is treated as reaching the extreme, not as a fault.
   * @param timeout_ms Window with no movement before declaring a stall.
   * @param min_counts Minimum encoder counts that qualify as movement (default 4).
   */
  void enableStallDetection(unsigned long timeout_ms, long min_counts = 4);

  /**
   * Disables stall detection.
   */
  void disableStallDetection();

  /**
   * Reports whether a stall fault is latched. While latched, run() holds the
   * motor braked until clearStall() is called.
   * @return True if stalled.
   */
  bool isStalled();

  /**
   * Clears a latched stall fault and resumes normal operation.
   */
  void clearStall();

  /**
   * Starts a non-blocking homing move: drives at a fixed PWM toward an extreme
   * until the matching endstop triggers or a stall is detected (whichever is
   * attached/enabled), then brakes and zeroes the encoder there. Software
   * travel limits are bypassed during the move, and any latched stall fault
   * is cleared (a stall during homing marks the extreme, not a fault).
   * Progressed by run(); poll isHoming() for completion and isHomed() for
   * success. Cancelled by stop().
   * @param direction Negative for the minimum-side extreme, positive for maximum-side.
   * @param pwm Drive PWM during homing (clamped to [pwm_skip, maxPWM]).
   * @param max_travel Failsafe: abort (unhomed) after this many encoder counts
   *        of travel, e.g. against a dead endstop switch. 0 = unbounded.
   * @return False if direction is 0 or no endstop/stall detection can detect the extreme.
   */
  bool startHoming(int8_t direction, uint8_t pwm, long max_travel = 0);

  /**
   * Reports whether a homing move is in progress.
   * @return True while homing.
   */
  bool isHoming();

  /**
   * Reports whether the last homing move completed successfully (encoder was
   * zeroed at a detected extreme). Cleared by startHoming(); stays false if
   * homing was cancelled, aborted by max_travel, or lost its termination
   * conditions mid-move. isHoming()==false alone does not imply success.
   * @return True if homed.
   */
  bool isHomed();

  /**
   * Provides debugging information.
   * @return A string with current status and PID parameters.
   */
  String getDebugInfo();

  /**
   * Generates a string formatted for Serial Plotter.
   * @param id Optional ID for the motor (default is -1 which means no ID).
   *        This is useful for distinguishing between multiple motors in the plotter.
   * @return A comma-separated string of PID setpoint, input, and output.
   */
  String getSerialPlotter(int id = -1);

private:
  // PID control variables:
  double _PID_setpoint, _PID_input, _PID_output;
  uint8_t _PWM_output;        // Computed PWM value
  uint8_t _pwm_skip;          // Minimum PWM to overcome low-power stall
  uint8_t _maxPWM;            // Maximum allowable PWM value
  unsigned int _position_accuracy; // Tolerance for position accuracy

  // Software travel limits:
  bool _limits_enabled;
  long _limit_min, _limit_max;

  // Physical endstops (nullptr = not attached):
  EndstopReadFunc endstopMin;
  EndstopReadFunc endstopMax;

  // Stall detection:
  bool _stall_enabled;
  bool _stalled;                  // Latched stall fault
  bool _driving;                  // Motor was driven last cycle (stall window validity)
  unsigned long _stall_timeout;   // ms without movement before declaring a stall
  long _stall_min_counts;         // Encoder counts that qualify as movement
  unsigned long _stall_last_time; // Start of the current no-movement window
  long _stall_last_pos;           // Encoder position at window start

  // Homing:
  bool _homing;
  bool _homed;             // Last homing move succeeded (encoder zeroed at extreme)
  int8_t _homing_dir;
  uint8_t _homing_pwm;
  long _homing_start_pos;  // Encoder position when homing started (max_travel bound)
  long _homing_max_travel; // Abort homing after this travel; 0 = unbounded

  long clampToLimits(long position); // Apply software travel limits
  void resetStallWindow();           // Restart the no-movement window
  bool stallDetected();              // True if the window expired without movement
  void runHoming();                  // Homing branch of run()
  void finishHoming(bool success);   // Halt; zero encoder on success, hold position on failure
  void haltMotor();                  // Brake, suspend the PID, end the drive cycle
  void watchStall(int16_t speed);    // Stall-fault check for a drive of speed

  // Function pointers for hardware abstraction:
  MotorWriteFunc motorWrite;
  MotorBrakeFunc motorBrake;
  EncoderReadFunc encoderRead;
  EncoderWriteFunc encoderWrite;
};

#endif // DCMotorServo_h

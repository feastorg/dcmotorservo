#include <Arduino.h>
#include "baseSpecs.h"
#include "DCMotorServo.h"
#include "DCMotorTacho.h"

// Create an instance of DCMotorServo using the shared wrappers.
DCMotorServo servo(wrapMotorWrite, wrapMotorBrake, wrapEncoderRead, wrapEncoderWrite);

// Create an instance of DCMotorTacho (closed-loop speed).
DCMotorTacho tacho(&servo, CPR, SPEED_INTERVAL);

// Speed loop settings, adjusted via serial commands.
float speedKp = SPEED_KP, speedKi = SPEED_KI;
float ffStatic = FF_STATIC, ffKv = FF_KV;

// Open-loop sweep: while sweepPWM is not 0 the motor is driven at that PWM
// with the speed loop off, and the steady speed is printed in counts/s.
int sweepPWM = 0;
long sweepCount = 0;
unsigned long sweepTime = 0;

void setup()
{
    Serial.begin(115200);
    // Initialize the motor driver.
    motorDriver.begin();

    servo.setMaxPWM(255);

    tacho.setSpeedRPM(0);
    tacho.setPIDTunings(speedKp, speedKi);
    tacho.setFeedforward(ffStatic, ffKv);
    tacho.setTrimLimit(TRIM_LIMIT);

    Serial.println("DCMotorTacho (Closed-Loop Speed) Example");
    Serial.println("Format: MeasuredSpeedRPM | DesiredSpeedRPM");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    if (sweepPWM != 0)
    {
        servo.drive(sweepPWM);
        // Print the speed in counts/s every 500 ms.
        if (millis() - sweepTime >= 500)
        {
            long count = servo.getActualPosition();
            double countsPerS = (count - sweepCount) * 1000.0 / (millis() - sweepTime);
            Serial.print("PWM ");
            Serial.print(sweepPWM);
            Serial.print(": ");
            Serial.print(countsPerS, 1);
            Serial.println(" counts/s");
            sweepCount = count;
            sweepTime = millis();
        }
    }
    else
    {
        // Run the speed loop.
        tacho.run();

        // Print status every 50ms.
        if (millis() - lastPrintTime > 50)
        {
            lastPrintTime = millis();
            Serial.print("MeasuredSpeedRPM:");
            Serial.print(tacho.getMeasuredSpeedRPM(), 2);
            Serial.print(", DesiredSpeedRPM:");
            Serial.println(tacho.getDesiredSpeedRPM(), 2);
        }
    }

    // Process serial commands for tuning.
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("SPEED="))
        {
            sweepPWM = 0;
            double rpm = command.substring(6).toFloat();
            tacho.setSpeedRPM(rpm);
            Serial.print("Desired Speed set to: ");
            Serial.println(rpm);
        }
        else if (command.startsWith("PWM="))
        {
            tacho.stop();
            sweepPWM = command.substring(4).toInt();
            sweepCount = servo.getActualPosition();
            sweepTime = millis();
            if (sweepPWM == 0)
                servo.stop();
            Serial.print("Open-loop PWM set to: ");
            Serial.println(sweepPWM);
        }
        else if (command.startsWith("SPEEDKP="))
        {
            speedKp = command.substring(8).toFloat();
            tacho.setPIDTunings(speedKp, speedKi);
            Serial.print("Speed Kp set to: ");
            Serial.println(speedKp);
        }
        else if (command.startsWith("SPEEDKI="))
        {
            speedKi = command.substring(8).toFloat();
            tacho.setPIDTunings(speedKp, speedKi);
            Serial.print("Speed Ki set to: ");
            Serial.println(speedKi);
        }
        else if (command.startsWith("FFSTATIC="))
        {
            ffStatic = command.substring(9).toFloat();
            tacho.setFeedforward(ffStatic, ffKv);
            Serial.print("Feedforward static PWM set to: ");
            Serial.println(ffStatic);
        }
        else if (command.startsWith("FFKV="))
        {
            ffKv = command.substring(5).toFloat();
            tacho.setFeedforward(ffStatic, ffKv);
            Serial.print("Feedforward kv set to: ");
            Serial.println(ffKv);
        }
        else if (command.startsWith("TRIM="))
        {
            double trim = command.substring(5).toFloat();
            tacho.setTrimLimit(trim);
            Serial.print("Trim limit set to: ");
            Serial.println(trim);
        }
        else if (command.startsWith("STOP"))
        {
            sweepPWM = 0;
            tacho.stop();
            Serial.println("Motor stopped.");
        }
        else
        {
            Serial.println("Invalid command. Commands: SPEED=, PWM=, SPEEDKP=, SPEEDKI=, FFSTATIC=, FFKV=, TRIM=, STOP");
        }
    }
}

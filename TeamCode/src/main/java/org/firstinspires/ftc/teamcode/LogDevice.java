package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Static routines that dump information about devices to the log file.
 *
 * These routines would usually be called during initialization
 *
 * Used to explore DcMotorEx configuration (e.g., PIDF coefficients)
 * Used to describe Servo
 *
 * TODO: It might be better to overload some of these methods and make use of device hierarchy
 */
class LogDevice {
    // String used for the log
    private static final String TAG = "LogDevice";

    /**
     * Log information about a DcMotorEx
     *
     * @param name describes which motor (usually the variable name)
     * @param motor specifies the motor to describe
     */
    static void logMotor(String name, DcMotorEx motor) {
        Log.d(TAG, "motor characteristics for " + name);

        // not very interesting: just says "Motor"
        Log.d(TAG, "  device name: " + motor.getDeviceName());

        // not very interesting: just says "Lynx"
        Log.d(TAG, "  manufacturer: " + motor.getManufacturer());

        // expected this to indicate HD Hex and Core Hex, but apparently a random integer
        Log.d(TAG, "  type: " + motor.getMotorType());

        // reports into which port the device is plugged
        Log.d(TAG, "  port: " + motor.getPortNumber());

        // reports direction
        Log.d(TAG, "  direction: " + motor.getDirection());

        // reports the power
        //   I expect this value to be zero.
        //   In a PIDF mode, power is used as a limit
        Log.d(TAG, "  power: " + motor.getPower());

        // reports current position (encoder value)
        Log.d(TAG, "  position: " + motor.getCurrentPosition());
        // (dcMotor) the target position
        Log.d(TAG, "    TargetPosition: " + motor.getTargetPosition());
        // (dcMotorEx) tolerance is 5 ticks.
        Log.d(TAG, "    TargetPositionTolerance = " + motor.getTargetPositionTolerance());

        // (dcMotorEx) report velocity (is this the current velocity or the target velocity?
        //   looking for analog of motor.setVelocity(v)
        //   so the interface is a bit screwy.
        //      setVelocity() -> setVelocityTarget()
        //      and there should be a getVelocityTarget()
        Log.d(TAG, "  velocity " + motor.getVelocity());

        // dump information about the PIDF coefficients
        // Velocity control
        //   4.96, 0.496, 0, 49.6
        logPIDF("  PIDF(rue) = ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        // Position control
        //   The run to position algorithm only makes use of P.
        //   See .setPIDFCoefficients()
        //   5, 0, 0, 0
        logPIDF("  PIDF(r2p) = ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    /**
     * Log information about PIDF coefficients
     * @param msg - string to identify which coefficients
     * @param pidf - the PIDF coefficients to dump
     */
    static void logPIDF(String msg, PIDFCoefficients pidf) {
        Log.d(TAG, msg + pidf.p + ", " + pidf.i + ", " + pidf.d + ", " + pidf.f +
                " algorithm: " + pidf.algorithm);
    }

    /**
     * Log information about a Servo
     *
     * @param name - usually the variable name for the servo
     * @param servo - the servo to describe
     */
    static void logServo(String name, Servo servo) {
        Log.d(TAG, "servo information for " + name);
        // not very interesting: just says "Servo"
        Log.d(TAG, "  device name: " + servo.getDeviceName());
        // not very interesting: just says "Lynx"
        Log.d(TAG, "  manufacturer: " + servo.getManufacturer());
        // reports into which port the servo is plugged
        Log.d(TAG, "  port number: " + servo.getPortNumber());
        // reports current position
        Log.d(TAG, "  position: " + servo.getPosition());
    }

}

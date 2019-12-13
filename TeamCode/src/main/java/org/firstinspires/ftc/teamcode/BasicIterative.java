/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 */

@TeleOp(name="testbot", group="Test")
public class BasicIterative extends OpMode
{
    // Declare OpMode members.

    // for Log.d() and friends, see https://developer.android.com/reference/android/util/Log.html
    private static final String TAG = "Testbot";
    // so use Log.d(TAG, <string>) to log debugging messages

    // is this necessary? Opmode.time and Opmode.getRuntime() (submillisecond accuracy)
    private ElapsedTime runtime = new ElapsedTime();

    // TODO: gyroscope mess
    // I thought I had this working, but apparently lost...
    // private Gyroscope imu;
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    // drive motors
    // abstract to a class (eg, Robot) where attributes can be static and shared by other Opmodes
    //   the class can have methods such as .setDrivePower()
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    // arm motor
    // abstract to a class (eg, Robot) where attributes can be static
    private DcMotorEx motorArm = null;

    // elevator motor
    // abstract to a class
    private DcMotorEx motorElevator = null;

    // abstract to a class (eg, Robot) where attributes can be static
    // and static methods can .setHook()
    private Servo servoHookLeft = null;
    private Servo servoHookRight = null;

    // abstract to a class (eg, Robot) where attributes can be static
    private Servo servoGrab = null;

    // robot parameters
    // abstract to a class (eg, Robot) where static parameters describe the robot
    // the wheel diameters
    private final double mWheelDiameterLeft = 0.090;
    private final double mWheelDiameterRight = 0.090;
    // half the distance between the wheels
    private final double distWheel = 0.305 / 2;

    // derived robot parameters
    // Distance per tick
    //   leaving the units vague at this point
    // Currently using direct drive with a CoreHex motor
    // The CoreHex motor has 4 ticks per revolution and is geared down by 72
    //   those attributes should be in the DcMotor class
    // The HD Hex Motor has 56 ticks per revolution
    //    the 20:1 is geared 20 to 1
    //    the 40:1 is geared 40 to 1
    // The HD Hex Motor is also used with the Ultraplanetary gearbox
    //    the 3:1 cartridge is actually 84:29 (2.9...)
    //    the 4:1 cartridge is actually 76:21 (3.6...)
    //    the 5:1 cartridge is actually 68:13 (5.2...)
    // The DcMotor class can allow some help
    //   MotorConfigurationType .getMotorType()
    //     MotorConfigurationType#getUnspecifiedMotorType()
    //       do not know where the enum is
    //   java.lang.String .getDeviceName() (not the config name)
    //   HardwareDevice.Manufacturer .getManufacturer()
    //     https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/HardwareMap.html
    //       possibly uninteresting
    //   DcMotorEx has .getVelocity(AngleUnit unit), so it presumably knows the ticks per revolution
    //     however, there is not a .getCurrentPostion(AngleUnit unit)
    private final double distpertickLeft = mWheelDiameterLeft * Math.PI / (4 * 72);
    private final double distpertickRight = mWheelDiameterRight * Math.PI / (4 * 72);

    // the robot pose
    // abstract to a class (eg, Robot) with static parameters
    //   that class can have .updatePose(), .getPose()
    //   such a step may allow the Pose to be carried over from Autonomous to Teleop
    //     Autonomous can set the initial pose
    //     When Teleop starts, it can use the existing Pose
    //        If there was no teleop, then initial Pose is random
    //        A button press during teleop's init_loop can set a known Pose
    private double xPose = 0.0;
    private double yPose = 0.0;
    private double thetaPose = 0.0;

    // encoder counts
    // abstract to a class coupled to the drive motors (eg, Robot) as static
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    private int cEncoderLeft;
    private int cEncoderRight;

    // REV 2m distance sensor and attack mode
    // Also Rev2mDistanceSensor
    private DistanceSensor sensorRange2m;
    private double distAttack = 0.0;
    private boolean bAttack = false;

    // Color distance sensor
    private DistanceSensor sensorDistance;
    private ColorSensor sensorColor;

    // REV touch sensor as digital channel
    private DigitalChannel digitalTouch;

    // try complicated initialization
    private int markovElevator = -1;

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     * @TODO Zero the arm position
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");
        telemetry.addData("Status", "Initializing");

        // for I2C busses
        // for glr
        //   I2C-Bus-0
        //     "imu"
        //     "sensorColorRange", Rev Color Sensor v3.
        //   I2C-Bus-3
        //      "rev2meter", REV 2M Distance Sensor

        // parameters for the imu
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // set typical parameters
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // deal with these parameters later
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // imu = hardwareMap.get(Gyroscope.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // start initializing
        telemetry.addData("IMU", "initialize");
        imu.initialize(parameters);

        // find the drive motors
        // abstract to a common Class (eg, Robot)
        // for SCHSConfig
        //    0: rightMotor
        //    1: leftMotor
        //    2: armExtenderMotor
        //    3: elevatorMotor
        leftDrive  = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightMotor");

        LogDevice.logMotor("motorLeft", leftDrive);
        LogDevice.logMotor("motorRight", rightDrive);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // DcMotor Direction also affects the encoder counts
        // remember the current encoder counts
        // Should always do this (even if not resetting the Pose)
        cEncoderLeft = leftDrive.getCurrentPosition();
        cEncoderRight = rightDrive.getCurrentPosition();

        // The arm motor
        //   See comments at https://ftc-tricks.com/dc-motors/
        //     .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //     .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //   abstract to a common class (eg, Robot)
        motorArm = hardwareMap.get(DcMotorEx.class, "armExtenderMotor");
        LogDevice.logMotor("motorArm", motorArm);
        // assume it is at position 0 right now
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Maybe use DcMotor.getCurrentPosition() as initial value?
        motorArm.setTargetPosition(0);
        // use the arm as a servo
        // target position must be set before RUN_TO_POSITION is invoked
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // do not ask for a lot of power yet
        motorArm.setPower(1.0);
        // choose FLOAT or BRAKE
        motorArm.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));

        // The elevator motor
        motorElevator = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        LogDevice.logMotor("motorElevator", motorElevator);
        // assume it is at position 0 right now
        motorElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setTargetPosition(0);
        motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorElevator.setPower(1.0);
        motorElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // The foundation hooks
        // for SCHSConfig
        //   0: leftHook
        //   1: rightHook
        //   2: grabberServo
        servoHookLeft = hardwareMap.get(Servo.class, "leftHook");
        servoHookRight = hardwareMap.get(Servo.class, "rightHook");

        // dump information about the servos
        LogDevice.logServo("servoHookLeft", servoHookLeft);
        LogDevice.logServo("servoHookRight", servoHookRight);

        // set hooks to known state
        setHookState(false);

        // The grabber actuator
        servoGrab = hardwareMap.get(Servo.class, "grabberServo");

        // find the REV 2m distance sensor
        sensorRange2m = hardwareMap.get(DistanceSensor.class, "rev2meter");

        // The color/distance sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        // touch sensor
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        // Tell the driver station that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // elevator initialization state
        markovElevator = -1;

        // update statistics vars
        cLoop = 0;
        timeLoop = time;

        Log.d(TAG, "init() complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("init", "looping; look for config info");

        // update statistics for loop period
        // TODO why is this loop taking 100 ms?
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

        // look at the imu
        if (imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "calibrated");
        } else {
            telemetry.addData("IMU", "calibrating");
        }

        // report digital touch sensor
        telemetry.addData("Touch", (digitalTouch.getState()) ? "no" : "yes");

        // report color
        // had used graph to scale red / 0.86, green / 0.98, and blue / 0.61
        // get values
        double tColor = getRuntime();
        int rColor = sensorColor.red();
        int gColor = sensorColor.green();
        int bColor = sensorColor.blue();
        int aColor = sensorColor.alpha();
        // the hue... in unknown units
        int hColor = sensorColor.argb();
        // time to get values is about 50 microseconds
        tColor = getRuntime() - tColor;

        telemetry.addData("raw RGB",
                String.format(Locale.US, "%d %d %d / %d %d in %8.02f ms",
                        rColor, gColor, bColor, aColor, hColor,
                        tColor));

        // measure distance in cm
        telemetry.addData("distance",
                String.format(Locale.US, "%.02f cm",
                        sensorDistance.getDistance(DistanceUnit.CM)));

        // try the 2m sensor
        double tau = getRuntime();
        double d = sensorRange2m.getDistance(DistanceUnit.CM);
        // raw time is about 17  to 20 ms
        tau = getRuntime() - tau;

        telemetry.addData("distance2m",
                String.format(Locale.US, "%8.02f cm, % 8.02f ms",d, tau * 1000));
        // Rev2mDistanceSensor specific methods.
        // or cast as more exotic sensor
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange2m;

        // this reports "ee"
        // telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        // this is always "false" (even with distance = 819cm (8 meters out)
        // telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        /* */
        // do complicated initialization of elevator
        switch (markovElevator) {
            case -1:
                if (gamepad1.a) {
                    // if the button is pressed, start initialization

                    // move the elevator up to clear elevator motor
                    // TODO make this an absolute position
                    motorElevator.setTargetPosition(500);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 0:
                // the elevator should be rising
                // when it gets high enough, start the next step
                if (motorElevator.getCurrentPosition() > 400) {
                    // extend the arm
                    motorArm.setTargetPosition(400);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 1:
                // the arm is extending
                // when it has gone far enough, start the next step
                if (motorArm.getCurrentPosition() > 300) {
                    // should be able to slowly lower the elevator onto the limit switch
                    // just bring it to 0 for now
                    motorElevator.setTargetPosition(0);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 2:
                // elevator is lowering onto switch
                if (motorElevator.getCurrentPosition() < 20) {
                    // figure we are done.
                    // send Elevator back up
                    motorElevator.setTargetPosition(300);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 3:
                // elevator is rising again
                if (motorElevator.getCurrentPosition() > 250) {
                    // retract arm
                    motorArm.setTargetPosition(0);

                    // terminate the calibration
                    markovElevator = -1;
                }
                break;
        }
        /**/
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Log.d(TAG, "start()");

        // reset the clock
        //   this may be superfluous because Opmode.time is reset at start of an Opmode
        //   Opmode.getRuntime() may also be reset
        //   In other words, I think Opmode supplies a reasonable time.
        runtime.reset();

        // reset timer statistics
        cLoop = 0;
        timeLoop = time;

        // may want to set the robot pose here...
        //   but more likely the Pose should carry over from last Opmode or
        //   be set during init_loop()

        // Start the logging of measured acceleration
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        Log.d(TAG, "start() complete");
    }

    /**
     * Set Hooks to a known state
     * Tte values are different, so there is probably bias in servo horn.
     * The servo horn has 25 teeth, so it can only be positioned to 360/25 = 14.4 degrees
     * Set for a small change now to avoid hitting the elevator winch.
     * TODO provide a tracking variable or read servo position
     * TODO remember the time the command was issue so completion can be estimated
     */
    private void setHookState (boolean state) {
        if (state) {
            // set the hook
            servoHookLeft.setPosition(0.5);     // larger is lower down
            servoHookRight.setPosition(0.55);    // smaller is lower down
        } else {
            // release the hook
            servoHookLeft.setPosition(0.0);
            servoHookRight.setPosition(1.0);
        }
    }

    /**
     * Update the robot pose.
     * Uses small angle approximations.
     * See COS495-Odometry by Chris Clark, 2011,
     * <a href="https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf">https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf</a>
     * TODO: Move to a common class (eg, Robot)
     */
    private void updateRobotPose() {
        // several calculations are needed

        // get the new encoder positions
        int cLeft = leftDrive.getCurrentPosition();
        int cRight = rightDrive.getCurrentPosition();

        // calculate the arc length deltas
        int dsLeft = cLeft - cEncoderLeft;
        int dsRight = cRight - cEncoderRight;

        // save the new encoder positions for the next time around
        cEncoderLeft = cLeft;
        cEncoderRight = cRight;

        double distL = dsLeft * distpertickLeft;
        double distR = dsRight * distpertickRight;

        // approximate the arc length as the average of the left and right arcs
        double ds = (distR + distL) / 2;
        // approximate the angular change as the difference in the arcs divided by wheel offset from
        // center of rotation.
        double dtheta = (distR - distL) / ( 2 * distWheel);

        // approximate the hypotenuse as just ds
        // approximate the average change in direction as one half the total angular change
        double dx = ds * Math.cos(thetaPose + 0.5 * dtheta);
        double dy = ds * Math.sin(thetaPose + 0.5 * dtheta);

        // update the current pose
        xPose = xPose + dx;
        yPose = yPose + dy;
        thetaPose = thetaPose + dtheta;

        // change radians to degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f", xPose, yPose, thetaPose * 180 / Math.PI);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

        // update the robot pose
        updateRobotPose();

        // query the imu
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // gravity  = imu.getGravity();

        // telemetry.addData("imu status", imu.getSystemStatus().toShortString());
        // telemetry.addData("imu calib", imu.getCalibrationStatus().toString());

        telemetry.addData("imu", "heading %.1f roll %.1f pitch %.1f",
                angles.firstAngle,
                angles.secondAngle,
                angles.thirdAngle);

        /*
        telemetry.addData("gravity", gravity.toString());
        telemetry.addData("gravmag", "%0.3",
                Math.sqrt(gravity.xAccel * gravity.xAccel +
                        gravity.yAccel * gravity.yAccel +
                        gravity.zAccel * gravity.zAccel));

         */

        // variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Drive should be abstracted to a class
        //   drive commands are often nonlinear
        //   sharing same nonlinearity across all Opmodes presents consistent interface to the driver
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        //   Uh, that's poor coding; have an attribute choose between the modes
        //      static boolean bTankMode = false;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // pushing joystick forward is negative y
        //   https://ftc-tricks.com/dc-motors/ :
        //   "Did you know that the Y-axis of the joysticks is negative when pushed up, and positive when pushed down?"
        //   Perhaps that comes from an airplane stick: pushing the stick forward causes plane to descend
        //   Gamepad joystick:
        //        -y
        //     -x  0  +x
        //        +y
        //       need to check that x values are not reversed
        // TOPDO: In Odemetry, angles are reported negative turning right is positive
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (bAttack) {
            // monitor if done
            if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                // the attack is done
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);

                // restore normal driving mode
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // attack is finished; resume normal driving
                bAttack = false;
                Log.d(TAG, "Attack finished");
            }
        } else {
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        // simple servo hacks
        if (gamepad1.right_bumper) {
            telemetry.addData("grab", "released");
            servoGrab.setPosition(0.1);
        } else {
            telemetry.addData("grab", "gripping");
            servoGrab.setPosition(0.6);
        }

        // set arm position hack
        // this needs a lot of work, but hardware has been removed
        // TODO: set scale
        // The arm is driven by a Core Hex motor with 288 counts per revolution.
        // The winch spool is made from 60-tooth gears.
        // The screws are at the 16 mm positions, but they do not form an equilateral triangle.
        // Distance will vary until that is fixed.
        // Mechanism is single stage with 1:1 spool to extension distance.
        // it is taking too long for isBusy() to report success, so just set the desired position.
        if (motorArm.isBusy()) {
            telemetry.addData("arm", "is busy");
            motorArm.setTargetPosition((int)(gamepad1.right_trigger * 500));
        } else {
            motorArm.setTargetPosition((int)(gamepad1.right_trigger * 500));
        }

        // set elevator position
        // TODO: set scale
        // the elevator is drivien by a Core Hex motor with 288 counts pre revolution.
        // the motor drives spools made from 45 tooth gears with screws at 3 8mm positions.
        // say a wrap averages 3 * 7/8 inches = 21/8 = 2 5/8 inches.
        // Thus 500 counts should be about 5 inches.
        // Elevator is continuous, so height is 1:1.
        // So 500 counts raises elevator about 5 inches.
        motorElevator.setTargetPosition((int)(gamepad1.left_trigger * 1500));

        // control the hooks
        if (gamepad1.left_bumper) {
            setHookState(true);
        } else {
            setHookState(false);
        }

        // try an attack mode
        if (gamepad1.dpad_down) {
            // run just once
            if (!bAttack) {
                Log.d(TAG, "Attack start");

                // calculate the attack distance
                distAttack = sensorRange2m.getDistance(DistanceUnit.CM);
                // should only attack if distance is reasonable
                if (distAttack < 60) {
                    // distance is less than 40 cm.
                    int cEncoder = -(int)((distAttack - 5) * 0.01 / distpertickLeft);

                    // set the target positions
                    leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + cEncoder);
                    rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + cEncoder);

                    leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftDrive.setPower(0.3);
                    rightDrive.setPower(0.3);
                }
                bAttack = true;
            }
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        // time and getRuntime() are high precision, but they are from the start of the opmode
        // (i think init sets time to zero, but it may be even earlier)
        // telemetry.addData("time", time);
        // telemetry.addData("getRuntime()", getRuntime());
        telemetry.addData("Attack", "%.2f cm", distAttack);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Log.d(TAG, "stop()");

        // turn off the drive motors
        //   abstract to a common class (eg, Robot)
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // make sure hooks are in known state
        setHookState(false);

        Log.d(TAG, "stop() complete");
    }

    // Read the battery voltage
    // Put this is the robot class
    // put the sensor in a class variable
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}

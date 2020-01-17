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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    private static final String TAG = "testbot";
    // so use Log.d(TAG, <string>) to log debugging messages

    // TODO: is this necessary? Opmode.time and Opmode.getRuntime() (submillisecond accuracy)
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
    // TODO: safe to remove
    private DcMotorEx motorArm = null;

    // elevator motor
    // TODO: safe to remove
    private DcMotorEx motorElevator = null;

    // REV 2m distance sensor and attack mode
    // Also Rev2mDistanceSensor
    // *** private DistanceSensor sensorRange2m;
    private double distAttack = 0.0;
    private boolean bAttack = false;

    // Color distance sensor
    //*** private DistanceSensor sensorDistance;
    //*** private ColorSensor sensorColor;

    // REV touch sensor as digital channel
    private DigitalChannel digitalTouch;

    // try complicated initialization
    private int markovElevator = -1;

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    // SCHSDrive (has drive motors and other stuff)
    private SCHSDrive schsdrive = null;

    // SCHSArm (has lift, extend, and hooks)
    private SCHSArm schsarm = null;

    // for tests
    private boolean brun = false;
    private boolean bspin = false;

    /**
     * Code to run ONCE when the driver hits INIT
     * TODO: Zero the arm position
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");
        telemetry.addData("Status", "Initializing");

        // the robot drive
        schsdrive = new SCHSDrive();
        schsdrive.init(hardwareMap, telemetry);

        // set the pose for testing
        schsdrive.setPoseInches(0,0,0);

        // TODO stop using local copies
        leftDrive = schsdrive.motorLeft;
        rightDrive = schsdrive.motorRight;

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

        // use the arm abstraction now
        schsarm = new SCHSArm();
        // The arm motor
        motorArm = schsarm.extendMotor;
        // The elevator motor
        motorElevator = schsarm.liftMotor;

        // set hooks to known state
        schsarm.setHookState(false);

        // find the REV 2m distance sensor
        // *** sensorRange2m = hardwareMap.get(DistanceSensor.class, "rev2meter");

        // The color/distance sensor
        //*** sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        //*** sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        // touch sensor
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        // elevator initialization state
        markovElevator = -1;

        // update statistics vars
        cLoop = 0;
        timeLoop = time;

        // Tell the driver station that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Log.d(TAG, "init() complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("init", "looping; look for config info");

        // update drive chassis
        schsdrive.init_loop();

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

        /* ***
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
        // A github issue says argb is not the hue but rather the alpha/R/G/B packed into bytes
        //    .see https://github.com/FIRST-Tech-Challenge/SkyStone/issues/140
        //   suggests
        //     float[] hsvValues = new float[3];
        //     Color.colorToHSV(color_sensor.argb(), hsvValues);
        //     float hue = hsvValues[0];  // [0, 360]
        //     float sat = hsvValues[1];  // [0, 1]
        //     float val = hsvValues[2];  // [0, 1]
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
*/
        /* */
        // do complicated initialization of elevator
        switch (markovElevator) {
            case -1:
                if (gamepad1.a) {
                    // if the button is pressed, start initialization

                    // move the elevator up to clear elevator motor
                    schsarm.setLiftTargetHeight(10.0);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 0:
                // the elevator should be rising
                // when it gets high enough, start the next step
                if (schsarm.getLiftCurrentHeight() > 6.0) {
                    // extend the arm
                    schsarm.setArmTargetExtension(4.0);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 1:
                // the arm is extending
                // when it has gone far enough, start the next step
                if (schsarm.getArmCurrentExtension() > 3.5) {
                    // should be able to slowly lower the elevator onto the limit switch
                    // just bring it to 4.0 for now
                    schsarm.setLiftTargetHeight(4.0);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 2:
                // elevator is lowering onto switch
                if (schsarm.getLiftCurrentHeight() < 4.5) {
                    // figure we are done.
                    // send Elevator back up
                    schsarm.setLiftTargetHeight(10.0);

                    // advance the state
                    markovElevator++;
                }
                break;

            case 3:
                // elevator is rising again
                if (schsarm.getLiftCurrentHeight() > 6.0) {
                    // retract arm
                    schsarm.setArmTargetExtension(0.0);

                    // terminate the calibration
                    markovElevator = -1;
                }
                break;
        }
        /**/

        // for debugging drive motors
        if (!bspin) {
            // not spinning -- look to start
            bspin = true;

            if (gamepad1.x) {
                // make the motors turn 10 revolutions
                int ticks = (int)(10 * schsdrive.ticksPerWheelRev);
                DcMotorEx leftDrive = schsdrive.motorLeft;
                DcMotorEx rightDrive = schsdrive.motorRight;

                schsdrive.motorLeft.setTargetPosition(
                        schsdrive.motorLeft.getCurrentPosition() + ticks);
                schsdrive.motorRight.setTargetPosition(
                        schsdrive.motorRight.getCurrentPosition() + ticks);

                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftDrive.setPower(0.8);
                rightDrive.setPower(0.8);
            }
        } else {
            if (! schsdrive.motorLeft.isBusy() && ! schsdrive.motorRight.isBusy()) {
                DcMotorEx leftDrive = schsdrive.motorLeft;
                DcMotorEx rightDrive = schsdrive.motorRight;

                // we are done
                bspin = false;

                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        if (!brun) {
            // try running
            brun = true;
            if (gamepad1.y) {
                // make the motors move 48 inches (two tiles)
                int ticks = schsdrive.ticksFromInches(48.0);
                DcMotorEx leftDrive = schsdrive.motorLeft;
                DcMotorEx rightDrive = schsdrive.motorRight;

                schsdrive.motorLeft.setTargetPosition(
                        schsdrive.motorLeft.getCurrentPosition() + ticks);
                schsdrive.motorRight.setTargetPosition(
                        schsdrive.motorRight.getCurrentPosition() + ticks);

                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftDrive.setPower(0.8);
                rightDrive.setPower(0.8);
            }
        } else {
            if (! schsdrive.motorLeft.isBusy() && ! schsdrive.motorRight.isBusy()) {
                DcMotorEx leftDrive = schsdrive.motorLeft;
                DcMotorEx rightDrive = schsdrive.motorRight;

                // we are done
                brun = false;

                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
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

        schsdrive.start();

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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

        // update robot position
        schsdrive.loop();

        // report position in meters and degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f",
                schsdrive.xPoseInches,
                schsdrive.yPoseInches,
                schsdrive.thetaPoseDegrees);

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
            schsarm.setGrabState(false);
        } else {
            telemetry.addData("grab", "gripping");
            schsarm.setGrabState(true);
        }

        // set arm position hack
        // it is taking too long for isBusy() to report success, so just set the desired position.
        // set 0 to 12 inches (method will clip)
        schsarm.setArmTargetExtension(gamepad1.right_trigger * 12.0);

        // set elevator height
        // set 0 to 30 inches (method will clip)
        schsarm.setLiftTargetHeight(gamepad1.left_trigger * 30.0);

        // control the hooks
        if (gamepad1.left_bumper) {
            schsarm.setHookState(true);
        } else {
            schsarm.setHookState(false);
        }

        // try an attack mode
        if (gamepad1.dpad_down) {
            // run just once
            if (!bAttack) {
                Log.d(TAG, "Attack start");

                // calculate the attack distance
                distAttack = 30; // ***  sensorRange2m.getDistance(DistanceUnit.CM);
                // should only attack if distance is reasonable
                if (distAttack < 60) {
                    // distance is less than 40 cm.
                    int cEncoder = schsdrive.ticksFromMeters((distAttack - 5) * 0.01);

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

        schsdrive.stop();

        // turn off the drive motors
        //   abstract to a common class (eg, Robot)
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // make sure hooks are in known state
        schsarm.setHookState(false);

        Log.d(TAG, "stop() complete");
    }
}

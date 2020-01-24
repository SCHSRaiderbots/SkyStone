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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * A Basic Teleop Mode
 *
 * Uses IMU to compare odometry angles
 * Has an attack mode that uses distance sensor to run up to a stone
 */

@TeleOp(name="testbot", group="Test")
public class TeleopBasic extends OpMode
{
    // for Log.d() and friends, see https://developer.android.com/reference/android/util/Log.html
    // so use Log.d(TAG, <string>) to log debugging messages
    private static final String TAG = "testbot";

    // TODO: is this necessary? Opmode.time and Opmode.getRuntime() (submillisecond accuracy)
    private ElapsedTime runtime = new ElapsedTime();

    // private Gyroscope imu;
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    // Attack mode
    private double distAttack = 0.0;
    private boolean bAttack = false;

    // try complicated initialization
    private int markovElevator = -1;

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    // RobotEx (has drive motors, arm, and other stuff)
    RobotEx robot = null;

    // for tests
    private boolean brun = false;
    private boolean bspin = false;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");
        telemetry.addData("Status", "Initializing");

        // the robot drive
        robot = new RobotEx();
        robot.init(hardwareMap, telemetry);

        // set the pose for testing
        robot.setPoseInches(0,0,0);

        // Set up the IMU

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

        // imu = hardwareMap.get(Gyroscope.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // start initializing
        telemetry.addData("IMU", "initialize");
        imu.initialize(parameters);

        // possibly initialize the arm
        if (robot.arm != null) {
            // there is an arm

            // Zero the arm position
            //   this should be done in autonomous, but may be a disaster during teleop.
            robot.arm.resetArmEncoders();

            // set hooks to known state
            robot.arm.setHookState(false);
        }

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
        robot.init_loop();

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

        // if the robot has an arm
        if (robot.arm != null) {
            // do complicated initialization of elevator
            switch (markovElevator) {
                case -1:
                    if (gamepad1.a) {
                        // if the button is pressed, start initialization

                        // move the elevator up to clear elevator motor
                        robot.arm.setLiftTargetHeight(10.0);

                        // advance the state
                        markovElevator++;
                    }
                    break;

                case 0:
                    // the elevator should be rising
                    // when it gets high enough, start the next step
                    if (robot.arm.getLiftCurrentHeight() > 6.0) {
                        // extend the arm
                        robot.arm.setArmTargetExtension(4.0);

                        // advance the state
                        markovElevator++;
                    }
                    break;

                case 1:
                    // the arm is extending
                    // when it has gone far enough, start the next step
                    if (robot.arm.getArmCurrentExtension() > 3.5) {
                        // should be able to slowly lower the elevator onto the limit switch
                         robot.arm.setLiftTargetHeight(robot.arm.LIFT_HEIGHT_MIN);

                        // advance the state
                        markovElevator++;
                    }
                    break;

                case 2:
                    // elevator is lowering onto switch
                    if (robot.arm.getLiftCurrentHeight() < robot.arm.LIFT_HEIGHT_MIN + 1.0) {
                        // figure we are done.
                        // send Elevator back up
                        robot.arm.setLiftTargetHeight(10.0);

                        // advance the state
                        markovElevator++;
                    }
                    break;

                case 3:
                    // elevator is rising again
                    if (robot.arm.getLiftCurrentHeight() > 6.0) {
                        // retract arm
                        robot.arm.setArmTargetExtension(0.0);

                        // terminate the calibration
                        markovElevator = -1;
                    }
                    break;
            }
        }
        /**/

        // for debugging drive motors

        // spin wheels 10 turns
        telemetry.addLine("Press X for 10 turns");
        if (!bspin) {
            // not spinning

            // should we start?
            if (gamepad1.x) {
                // set spin mode
                bspin = true;

                // make the motors turn 10 revolutions
                int ticks = (int)(10 * robot.ticksPerWheelRev);

                robot.motorLeft.setTargetPosition(
                        robot.motorLeft.getCurrentPosition() + ticks);
                robot.motorRight.setTargetPosition(
                        robot.motorRight.getCurrentPosition() + ticks);

                robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.setDrivePower(0.8, 0.8);
            }
        } else {
            // we are spinning the wheels; check if they have stopped
            if (! robot.motorLeft.isBusy() && ! robot.motorRight.isBusy()) {
                // the motors have stopped, so we are done
                bspin = false;

                // restore the mode
                robot.setDrivePower(0.0, 0.0);
                robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        // drive forward 48 inches
        telemetry.addLine("Press Y to move 48 inches");
        if (!brun) {
            // not running

            // should we start running?
            if (gamepad1.y) {
                // set running mode
                brun = true;

                // make the motors move 48 inches (two tiles)
                int ticks = robot.ticksFromInches(48.0);

                robot.motorLeft.setTargetPosition(
                        robot.motorLeft.getCurrentPosition() + ticks);
                robot.motorRight.setTargetPosition(
                        robot.motorRight.getCurrentPosition() + ticks);

                // command the motors
                robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.setDrivePower(0.8, 0.8);
            }
        } else {
            // we are running a set diestance
            // check if the run is finished
            if (! robot.motorLeft.isBusy() && ! robot.motorRight.isBusy()) {
                // we have reached the desired position, so we are done
                brun = false;

                // command the motors
                robot.setDrivePower(0.0, 0.0);
                robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        robot.start();

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
        robot.loop();

        // report position in meters and degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f",
                robot.xPoseInches,
                robot.yPoseInches,
                robot.thetaPoseDegrees);

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

        // apply square law
        drive = drive * Math.abs(drive);
        turn = turn * Math.abs(turn);

        // TODO set Power is not linear with rwe
        // running using encoder causes attack to fail
        // OOPS, this causes attack to fail
        // robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (bAttack) {
            // monitor if done
            if (!robot.motorLeft.isBusy() && !robot.motorRight.isBusy()) {
                // the attack is done
                // robot.setDrivePower(0.0, 0.0);

                // restore normal driving mode
                robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // attack is finished; resume normal driving
                bAttack = false;

                Log.d(TAG, "Attack finished");
            }
        } else {
            // Send calculated power to wheels
            robot.setDrivePower(leftPower, rightPower);
        }

        // if there is an arm
        if (robot.arm != null) {
            // set arm position hack
            // it is taking too long for isBusy() to report success, so just set the desired position.
            // set 0 to 12 inches (method will clip)
            robot.arm.setArmTargetExtension(gamepad1.right_trigger * 12.0);

            // set elevator height
            // set 0 to 30 inches (method will clip)
            robot.arm.setLiftTargetHeight(gamepad1.left_trigger * 30.0);

            // operate the grabber
            if (gamepad1.right_bumper) {
                telemetry.addData("grab", "released");
                robot.arm.setGrabState(false);
            } else {
                telemetry.addData("grab", "gripping");
                robot.arm.setGrabState(true);
            }

            // control the hooks
            if (gamepad1.left_bumper) {
                robot.arm.setHookState(true);
            } else {
                robot.arm.setHookState(false);
            }
        }

        // check for an attack mode
        telemetry.addLine("Press dpad_down for attack");
        if (gamepad1.dpad_down) {
            // run just once
            if (!bAttack) {
                Log.d(TAG, "Attack start");

                // measure the attack distance
                distAttack = robot.inchRangeMeasurement();

                // attack if distance is reasonable
                if (distAttack < 30.0) {
                    // distance is reasonable
                    // calculate distance to move
                    int cEncoder = robot.ticksFromInches((distAttack - 3.0));

                    // Log.d(TAG, "Current run mode is " + robot.motorLeft.getMode());

                    // set the target positions
                    // Log.d(TAG, "Set attack distance");
                    robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition() + cEncoder);
                    robot.motorRight.setTargetPosition(robot.motorRight.getCurrentPosition() + cEncoder);

                    // use 0.8 for Riley; 0.3 for 2020 Bot
                    Log.d(TAG, "Set attack drive power");
                    robot.setDrivePower(0.8, 0.8);

                    // Log.d(TAG, "Set attack run mode");
                    robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // remember we are attacking
                    bAttack = true;
                } else {
                    Log.d(TAG, "Attack declines");
                }
            }
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        // time and getRuntime() are high precision, but they are from the start of the opmode
        // (i think init sets time to zero, but it may be even earlier)
        // telemetry.addData("time", time);
        // telemetry.addData("getRuntime()", getRuntime());
        telemetry.addData("Attack", "%.2f in", distAttack);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Log.d(TAG, "stop()");

        // turn off the robot
        robot.stop();

        // if there is an arm, clean it up
        if (robot.arm != null) {
            // make sure hooks are in known state
            robot.arm.setHookState(false);
        }

        Log.d(TAG, "stop() complete");
    }
}

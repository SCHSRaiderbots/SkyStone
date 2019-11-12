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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class BasicIterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // I thought I had this working, but apparently lost...
    // private Gyroscope imu;
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    // drive motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private Servo servoHookLeft = null;
    private Servo servoHookRight = null;
    private Servo servoGrab = null;

    // robot parameters
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
    //   that should be in the DcMotor class
    private final double distpertickLeft = mWheelDiameterLeft * Math.PI / (4 * 72);
    private final double distpertickRight = mWheelDiameterRight * Math.PI / (4 * 72);

    // the robot pose
    private double xPose = 0.0;
    private double yPose = 0.0;
    private double thetaPose = 0.0;

    // encoder counts
    private int cEncoderLeft;
    private int cEncoderRight;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // parameters for the imu
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // imu = hardwareMap.get(Gyroscope.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // start initializing
        imu.initialize(parameters);

        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // I do not know if the Direction also affects the encoder counts
        // remember the current encoder counts
        cEncoderLeft = leftDrive.getCurrentPosition();
        cEncoderRight = rightDrive.getCurrentPosition();

        // The foundation hooks
        servoHookLeft = hardwareMap.get(Servo.class, "hookLeft");
        servoHookRight = hardwareMap.get(Servo.class, "hookRight");

        // The grabber actuator
        servoGrab = hardwareMap.get(Servo.class, "servoGrab");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("init", "looping; look for config info");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // reset the clock
        runtime.reset();

        // may want to set the robot pose here...

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /**
     * Update the robot pose.
     * Uses small angle approximations.
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
        // update the robot pose
        updateRobotPose();

        // query the imu
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        telemetry.addData("imu status", imu.getSystemStatus().toShortString());
        telemetry.addData("imu calib", imu.getCalibrationStatus().toString());

        telemetry.addData("imu", "heading %.1f roll %.1f pitch %.1f",
                angles.firstAngle,
                angles.secondAngle,
                angles.thirdAngle);
        telemetry.addData("gravity", gravity.toString());
        telemetry.addData("gravmag", "%0.3",
                Math.sqrt(gravity.xAccel * gravity.xAccel +
                        gravity.yAccel * gravity.yAccel +
                        gravity.zAccel * gravity.zAccel));

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // simple servo hacks
        if (gamepad1.dpad_up) {
            telemetry.addData("grab", "released");
            servoGrab.setPosition(0.1);
        } else {
            telemetry.addData("grab", "gripping");
            servoGrab.setPosition(0.6);
        }

        // control the hooks
        if (gamepad1.left_bumper) {
            telemetry.addData("hook", "set hook");
            servoHookLeft.setPosition(0.4);
            servoHookRight.setPosition(0.6);
        } else {
            telemetry.addData("hook", "release hook");
            servoHookLeft.setPosition(0.0);
            servoHookRight.setPosition(1.0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

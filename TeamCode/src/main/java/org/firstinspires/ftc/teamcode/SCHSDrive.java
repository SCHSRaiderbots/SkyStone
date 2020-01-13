package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDrive {
    // the hardware map
    HardwareMap hardwareMap;

    // information to report home
    Telemetry telemetry = null;

    // battery voltage; used to warn of a low battery
    double voltageBattery = 0.0;

    // the drive motors
    protected DcMotorEx motorLeft;
    protected DcMotorEx motorRight;

    // robot parameters
    // the wheel diameters are 90mm nominal
    private double mWheelDiameterLeft = 0.090;
    private double mWheelDiameterRight = 0.090;
    
    // half the distance between the wheels
    // the new wheel separation 13 + 15/16
    private double distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

    // The CoreHex motor has 4 ticks per revolution and is geared down by 72
    //   those attributes should be in the DcMotor class

    // The HD Hex Motor has 56 ticks per revolution
    //    or so claims http://www.revrobotics.com/content/docs/HDMotorEncoderGuide.pdf
    //    the 20:1 is geared 20 to 1
    //    the 40:1 is geared 40 to 1
    // The HD Hex Motor is also used with the Ultraplanetary cartridges
    //    the 3:1 cartridge is actually 84:29 (2.9...)
    //    the 4:1 cartridge is actually 76:21 (3.6...)
    //    the 5:1 cartridge is actually 68:13 (5.2...)
    private final double HD_HEX_GEAR_CART_3_1 = 84.0/29.0;
    private final double HD_HEX_GEAR_CART_4_1 = 76.0/21.0;
    private final double HD_HEX_GEAR_CART_5_1 = 68.0/13.0;

    // calculate the wheel's ticks per revolution
    //   Apparently, we only see 1/2 the counts that REV claims (28 instead of 56)
    double ticksPerWheelRev = (56/2) * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

    // derived robot parameters
    // Distance per tick
    //   leaving the units vague at this point

    // the distance per tick for each wheel = circumference / ticks
    private double distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
    private double distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);

    // the robot pose
    //   can have .updatePose(), .getPose(), .setPose()
    //   using static would allow the Pose to be carried over from Autonomous to Teleop
    //     Autonomous can set the initial pose
    //     When Teleop starts, it can use the existing Pose
    //        If there was no teleop, then initial Pose is random
    //        A button press during teleop's init_loop can set a known Pose
    double xPose = 0.0;
    double yPose = 0.0;
    double thetaPose = 0.0;

    // encoder counts
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    private int cEncoderLeft;
    private int cEncoderRight;

    // Anisha's variables
    private int leftEncoderTarget;
    private int rightEncoderTarget;

    /**
     * Called during an OpMode init() routine.
     * gets the drive motors setup
     * @param hdmap the FTC hardwareMap of devices
     * @param telem the telemetry object
     */
    public void init(HardwareMap hdmap, Telemetry telem) {
        // save the hardware map for future reference
        hardwareMap = hdmap;

        // save telemetry for future reference (may be null)
        telemetry = telem;

        // TODO: Check the firmware revisions on all Expansion Hubs
        //   should be 1.8.2
        //   17 December 2019: Updated Expansion Hub firmware to 1.8.2
        // would like to raise an alarm if the Expansion Hub is out of date

        // get the battery voltage
        //   measure this voltage just once
        //   we do not need an update in the middle of a match
        voltageBattery = getBatteryVoltage();

        // find the drive motors
        // for SCHSConfig
        //    0: rightMotor
        //    1: leftMotor
        //    2: armExtenderMotor
        //    3: elevatorMotor

        // get the drive motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // 16 December 2019: PIDF coefficients could not be set to 5,0,0,0 in logcat
        //   PIDF(rue) = 9.999847412109375, 2.9999542236328125, 0.0, 0.0
        //   PIDF(r2p) = 9.999847412109375, 0.04998779296875, 0.0, 0.0
        // still get an error message
        //   eg, 2019-12-17 10:12:06.021 13370-13425/com.qualcomm.ftcrobotcontroller W/LynxMotor: not supported: setPIDFCoefficients(0, RUN_TO_POSITION, PIDFCoefficients(p=5.000000 i=0.000000 d=0.000000 f=0.000000 alg=PIDF))
        // but report is now
        //   PIDF(rue) = 4.9600067138671875, 0.496002197265625, 0.0, 49.600006103515625 algorithm: PIDF
        //   PIDF(r2p) = 5.0, 0.0, 0.0, 0.0 algorithm: PIDF
        LogDevice.dump("motorLeft", motorLeft);
        LogDevice.dump("motorRight", motorRight);

        // remember the current encoder counts to do odometry
        // DcMotor Direction also affects the encoder counts
        // remember the current encoder counts
        // Should always do this (even if not resetting the Pose)
        cEncoderLeft = motorLeft.getCurrentPosition();
        cEncoderRight = motorRight.getCurrentPosition();
    }

    /**
     * old name for routine
     * @deprecated
     * @param hardwareMap
     */
    public void initialize(HardwareMap hardwareMap) {
        init(hardwareMap, null);
    }

    /**
     * Want to use this code with last year's robot, but it has different parameters
     */
    public void setRobot2019() {
        // set the wheel diameters
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        distWheel =  0.305 / 2;

        // ticks per wheel revolution
        // CoreHex motor...
        ticksPerWheelRev = 4 * 72;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * An OpMode should call this during its init_loop() method
     */
    public void init_loop() {
        // check robot health
        if (telemetry != null && voltageBattery < 11.5) {
            telemetry.addLine("RECHARGE or REPLACE BATTERY");
        }
    }

    /**
     * An OpMode should call this during its start() method
     */
    public void start() {

    }

    /**
     * An OpMode should call this during its loop() method
     */
    public void loop() {
        // Update the robot's position
        updateRobotPose();
    }

    /**
     * An OpMode should call this during its stop() method
     */
    public void stop() {
        // turn off the drive motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    /**
     * @deprecated public method should not reset encoders
     */
    public void resetEncoders(){
        setDrivePower(0,0);
        setDriveMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDrivePower(double powerLeft, double powerRight){
        motorLeft.setPower(Range.clip(powerLeft, -1, 1));
        motorRight.setPower(Range.clip(powerRight, -1, 1));
    }

    /**
     * @deprecated why is this be needed? Is it trying to force completion?
     */
    public void synchEncoders() {
        //	get and set the encoder targets
        leftEncoderTarget = motorLeft.getCurrentPosition();
        rightEncoderTarget = motorRight.getCurrentPosition();
    }

    public void setDriveMode(RunMode mode) {
        // Ensure the motors are in the correct mode.
        if (motorLeft.getMode() != mode)
            motorLeft.setMode(mode);

        if (motorRight.getMode() != mode)
            motorRight.setMode(mode);
    }

    public void useConstantSpeed() {
        motorLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorRight.setMode(RunMode.RUN_USING_ENCODER);
    }

    /**
     * @deprecated public methods should use standard units
     * @return
     */
    public boolean encodersAtZero() {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }

    /**
     * @deprecated public method should use standard units
     * @param leftEncoder ticks to advance left wheel encoder
     * @param rightEncoder ticks to advance right wheel encoder
     */
    public void addEncoderTarget(int leftEncoder, int rightEncoder){
        motorLeft.setTargetPosition(leftEncoderTarget += leftEncoder);
        motorRight.setTargetPosition(rightEncoderTarget += rightEncoder);
    }

    public double[] arcTurnPower(int arcRadius, double arcAngle, double arcTime){
        double outerRadius = 0;
        double innerRadius = 0;
        double innerTurnPower = 0;
        double outerTurnPower = 0;
        double[] turnPower = new double[2];

        outerRadius = arcRadius + (0.5*ROBOT_WIDTH);
        innerRadius = arcRadius + (0.5*ROBOT_WIDTH);

        double innerTurnSpeed = ((arcAngle)/(arcTime))*(innerRadius);
        double outerTurnSpeed = ((arcAngle)/(arcTime))*(outerRadius);

        innerTurnPower = innerTurnSpeed/ROBOT_MAX_SPEED;
        outerTurnPower = outerTurnSpeed/ROBOT_MAX_SPEED;

        turnPower[0] = innerTurnPower;
        turnPower[1] = outerTurnPower;

        return turnPower;
    }

    public double[] arcTurnDist(int arcRadius, double arcAngle) {
        double[] turnDist = new double[2];

        double outerRadius = arcRadius + (0.5*ROBOT_WIDTH);
        double innerRadius = arcRadius + (0.5*ROBOT_WIDTH);

        double outerTurnDist = outerRadius*arcAngle;
        double innerTurnDist = innerRadius*arcAngle;

        turnDist[0] = innerTurnDist;
        turnDist[1] = outerTurnDist;

        return turnDist;
    }

    /**
     * @deprecated public method should use standard units
     * @return current position for left wheel encoder
     */
    public double getLeftPosition(){
        return (motorLeft.getCurrentPosition());
    }

    /**
     * @deprecated public method should use standard units
     * @return current position for right wheel encoder
     */
    public double getRightPosition() {
        return (motorRight.getCurrentPosition());
    }

    /**
     * @deprecated public method should use standard units; use DcMotorEx
     * @return target position for left wheel encoder
     */
    public int getLeftEncoderTarget() {
        return leftEncoderTarget;
    }

    /**
     * @deprecated public method should use standard units; use DcMotorEx
     * @return target position for right wheel encoder
     */
    public int getRightEncoderTarget() {
        return rightEncoderTarget;
    }

    /**
     * Calculate the number of encoderticks to move the robot.
     * Uses just the left wheel
     * @param mDist distance in meters
     * @return encoder ticks to travel that distance
     */
    int ticksFromMeters(double mDist) {
        return (int)(mDist / distpertickLeft);
    }

    /**
     * Calculate the number of encoderticks to move the robot.
     * Uses just the left wheel
     * @param inDist distance in inches
     * @return encoder ticks to travel that distance
     */
    int ticksFromInches(double inDist) {
        return (int)(inDist * 0.0254 / distpertickLeft);
    }

    /**
     * Update the robot pose.
     * Uses small angle approximations.
     * See COS495-Odometry by Chris Clark, 2011,
     * <a href="https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf">https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf</a>
     */
    private void updateRobotPose() {
        // several calculations are needed

        // get the new encoder positions
        int cLeft = motorLeft.getCurrentPosition();
        int cRight = motorRight.getCurrentPosition();

        // calculate the arc length deltas
        int dsLeft = cLeft - cEncoderLeft;
        int dsRight = cRight - cEncoderRight;

        // save the new encoder positions for the next time around
        cEncoderLeft = cLeft;
        cEncoderRight = cRight;

        // calculate the distance the wheels moved
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
        // telemetry.addData("pose", "%8.2f %8.2f %8.2f", xPose, yPose, thetaPose * 180 / Math.PI);
    }

    /**
     * Read the battery voltage from all available voltage sensors
     * @return the minimum battery voltage or positive infinity
     */
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



package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDrive {

    // TODO: merge code from SCHSDrive
    private SCHSDcMotor driveMotors;
    private DcMotor motorLeft;
    protected DcMotor motorRight;

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
    private final double HD_HEX_3_1 = 84.0/29.0;
    private final double HD_HEX_4_1 = 76.0/21.0;
    private final double HD_HEX_5_1 = 68.0/13.0;

    private final double ticksPerRev = 56 * HD_HEX_5_1 * HD_HEX_4_1;

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
    private final double distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerRev);
    private final double distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerRev);

    // the robot pose
    // abstract to a class (eg, Robot) with static parameters
    //   that class can have .updatePose(), .getPose()
    //   such a step may allow the Pose to be carried over from Autonomous to Teleop
    //     Autonomous can set the initial pose
    //     When Teleop starts, it can use the existing Pose
    //        If there was no teleop, then initial Pose is random
    //        A button press during teleop's init_loop can set a known Pose
    double xPose = 0.0;
    double yPose = 0.0;
    double thetaPose = 0.0;

    // encoder counts
    // abstract to a class coupled to the drive motors (eg, Robot) as static
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    private int cEncoderLeft;
    private int cEncoderRight;

    private int leftEncoderTarget;
    private int rightEncoderTarget;

    /**
     * Called during an OpMode init() routine.
     * gets the drive motors setup
     * @param hardwareMap
     */
    public void initialize(HardwareMap hardwareMap) {
        driveMotors = new SCHSDcMotor();
        driveMotors.initialize(hardwareMap);

        motorLeft = driveMotors.getMotorleft();
        motorRight = driveMotors.getMotorRight();

        cEncoderLeft = motorLeft.getCurrentPosition();
        cEncoderRight = motorRight.getCurrentPosition();
    }

    /**
     * An OpMode should call this during its init_loop() method
     */
    public void init_loop() {
        // TODO: complain if battery voltage is low
    }

    /**
     * An OpMode should call this during its stop() method
     */
    public void start() {

    }

    /**
     * An OpMode should call this during its loop() method
     */
    public void loop() {
        // Update position
        updateRobotPose();
    }

    /**
     * An OpMode should call this during its stop() method
     */
    public void stop() {

    }

    public void resetEncoders(){
        setDrivePower(0,0);
        setDriveMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDrivePower(double powerLeft, double powerRight){
        motorLeft.setPower(Range.clip(powerLeft, -1, 1));
        motorRight.setPower(Range.clip(powerRight, -1, 1));
    }

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

    public boolean encodersAtZero() {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }

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

    public double getLeftPosition(){
        return (motorLeft.getCurrentPosition());
    }

    public double getRightPosition() {
        return (motorRight.getCurrentPosition());
    }

    public int getLeftEncoderTarget() {
        return leftEncoderTarget;
    }

    public int getRightEncoderTarget() {
        return rightEncoderTarget;
    }

    /**
     * Update the robot pose.
     * Uses small angle approximations.
     * See COS495-Odometry by Chris Clark, 2011,
     * <a href="https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf">https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf</a>
     * TODO: Move to a common class (eg, Robot)
     */
    private void updateRobotPose() {
        // TODO: these should be local variables
        DcMotor leftDrive = motorLeft;
        DcMotor rightDrive = motorRight;

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
        // telemetry.addData("pose", "%8.2f %8.2f %8.2f", xPose, yPose, thetaPose * 180 / Math.PI);
    }


}



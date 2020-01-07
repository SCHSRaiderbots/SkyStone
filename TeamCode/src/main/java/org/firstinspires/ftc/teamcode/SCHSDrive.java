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

    private SCHSDcMotor driveMotors;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private int leftEncoderTarget;
    private int rightEncoderTarget;

    public void initialize(HardwareMap hardwareMap) {
        driveMotors = new SCHSDcMotor();
        driveMotors.initialize(hardwareMap);

        motorLeft = driveMotors.getMotorleft();
        motorRight = driveMotors.getMotorRight();
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
}



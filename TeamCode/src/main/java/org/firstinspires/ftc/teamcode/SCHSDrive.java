package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDrive {

    private SCHSDcMotor driveMotors;
    private DcMotor motorLeft;
    protected DcMotor motorRight;

    private int leftEncoderTarget;
    private int rightEncoderTarget;

    private BNO055IMU.Parameters gyroParameters;
    private BNO055IMU imu;

    protected boolean isMoveDone;

    public void initialize(HardwareMap hardwareMap) {
        driveMotors = new SCHSDcMotor();
        driveMotors.initialize(hardwareMap);

        motorLeft = driveMotors.getMotorleft();
        motorRight = driveMotors.getMotorRight();

        gyroParameters = new BNO055IMU.Parameters();
        isMoveDone = false;

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        while (imu.isGyroCalibrated())  {
            Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyro is calibrating");
            sleep(50);
        }

        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyro done calibrating");

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

    //moves straight using gyro
    public void moveStraightWithGyro(double powerStart, double desiredPositionLeft, double desiredPositionRight) {
        double currAngle = 0;

        Log.d("Status" , "SCHSDrive:moveStraightWithGyro: Enter Method");

        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSDrive:moveStraightWithGyro: currAngle " + currAngle);

        gyroDrive(powerStart, desiredPositionLeft, desiredPositionRight, currAngle); //desired position in inches
        Log.d("Status" , "SCHSDrive:moveStraightWithGyro: gyroDrive finished");

        double finalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSDrive:moveStraightWithGyro: finalAngle " + finalAngle);

        isMoveDone = true;
    }

    //aligns using gyro angles
    public void gyroDrive(double speed, double distanceLeft, double distanceRight, double currentAngle) {
        int     newLeftTarget = 0;
        int     newRightTarget = 0;
        double  max = 0;
        double  error = 0;
        double  steer = 0;
        double  leftSpeed = 0;
        double  rightSpeed = 0;
        double countsPerInch = 0;
        double slowFactor = 0;
        long startTime = System.currentTimeMillis();

        Log.d("Status" , "SCHSDrive:gyroDrive: initial left position " + motorLeft.getCurrentPosition());
        Log.d("Status" , "SCHSDrive:gyroDrive: initial right position " + motorRight.getCurrentPosition());


        //converting inches to encoder values
        //countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        int encoderValue = (int) (COUNTS_PER_INCH * distanceLeft);

        Log.d("Status" , "SCHSDrive:gyroDrive: encoder value " + encoderValue);

        double temp = COUNTS_PER_INCH * distanceLeft;

        // 0201 org
        // slowFactor = (Math.abs(temp) + 309)/1550;
        slowFactor = (Math.abs(temp) + 309)/1550;
        Log.d("Status" , "SCHSDrive:gyroDrive: slowFactor " + slowFactor);

        // Determine new target position, and pass to motor controller
        newLeftTarget = motorLeft.getCurrentPosition() + encoderValue;
        newRightTarget = motorRight.getCurrentPosition() + encoderValue;
        Log.d("Status" , "SCHSDrive:gyroDrive: newLeftTarget " + newLeftTarget);
        Log.d("Status" , "SCHSDrive:gyroDrive: newRightTarget " + newRightTarget);

        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        // Set Target and Turn On RUN_TO_POSITION
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), -1.0, 1.0);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
        Log.d("Status" , "SCHSDrive:gyroDrive: speed " + speed);

        double PCoeff = 0.035;
        // keep looping while we are still active, and BOTH motors are running.
        while (motorLeft.isBusy() && motorRight.isBusy()) {

            Log.d("Status" , "SCHSMotor:gyroDrive:start of loop");

            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects left" + motorLeft.getCurrentPosition());
            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects right" + motorRight.getCurrentPosition());

            // adjust relative speed based on heading error.
            error = getError(currentAngle);
            Log.d("Status" , "SCHSDrive:gyroDrive: error " + error);

            steer = getSteer(error, PCoeff);
            Log.d("Status" , "SCHSDrive:gyroDrive: steer " + steer);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distanceLeft < 0) {
                steer *= -1.0;
            }

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            int distanceMovedLeft = motorLeft.getCurrentPosition();
            int distanceMovedRight = motorRight.getCurrentPosition();

            /*
            if (distanceMovedLeft >= slowFactor * Math.abs(newLeftTarget)|| distanceMovedRight >= slowFactor * Math.abs(newRightTarget)) {
                PCoeff = 0.75 * PCoeff;
                Log.d("Status" , "SCHSMotor:gyroDrive: PCoeff modified slowfactor " + PCoeff);
            }*/

//             feb 1st org
//             if (distanceMovedLeft >= slowFactor * Math.abs(newLeftTarget) || distanceMovedRight>= slowFactor * Math.abs(newRightTarget)) {

            if (Math.abs(distanceMovedLeft) >= slowFactor * Math.abs(newLeftTarget) || Math.abs(distanceMovedRight)>= slowFactor * Math.abs(newRightTarget)) {
                speed = 0.75 * speed;
                Log.d("Status" , "SCHSDrive:gyroDrive: speed modified slowfactor " + speed);
            }

            // if reached the desired position, exit while loop. Helps to stop turning at end of motion.
            if (distanceMovedLeft >= Math.abs(newLeftTarget) || distanceMovedRight >= Math.abs(newRightTarget)) {
                Log.d("Status" , "SCHSDrive:gyroDrive: Position reached. Break while");
                break;
            }

            Log.d("Status" , "SCHSDrive:gyroDrive:leftSpeed " + leftSpeed);
            Log.d("Status" , "SCHSDrive:gyroDrive:rightSpeed " + rightSpeed);

            Log.d("Status", "SCHSDrive:gyroDrive:before gyro corrects left " + motorLeft.getCurrentPosition());
            Log.d("Status", "SCHSDrive:gyroDrive:before gyro corrects right " + motorRight.getCurrentPosition());

            motorLeft.setPower(leftSpeed);
            motorRight.setPower(rightSpeed);

            Log.d("Status", "SCHSDrive:gyroDrive:after gyro corrects left " + motorLeft.getCurrentPosition());
            Log.d("Status", "SCHSDrive:gyroDrive:after gyro corrects right " + motorRight.getCurrentPosition());

            if ((Math.abs(getLeftPosition() - newLeftTarget) < 45) ||
                    (Math.abs(getRightPosition() - newRightTarget) < 45) /*|| System.currentTimeMillis() - startTime >= 6500*/)  {
                Log.d("Status", "SCHSDrive:gyroDrive: break from while loop");
                break;
            }
        }// end of while
    }

    public double getError(double startAngle) {

        double robotError = 0;

        // calculate error in -179 to +180 range  (
        //robotError = (startAngle + targetAngle) - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        robotError = startAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) {
            robotError -= 360;
        }

        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}


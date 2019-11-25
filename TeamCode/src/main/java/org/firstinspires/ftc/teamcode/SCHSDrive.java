package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDrive {

    private SCHSDcMotor driveMotors;
    private BNO055IMU.Parameters gyroParameters;
    private BNO055IMU imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    public void initialize(HardwareMap hardwareMap) {

        driveMotors = new SCHSDcMotor();
        driveMotors.initialize(hardwareMap);

        motorLeft = driveMotors.getMotorleft();
        motorRight = driveMotors.getMotorRight();

        gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);


    }

    //moves straight using gyro
    public void moveStraightWithGyro(double powerStart, int desiredPosition) {
        double currAngle = 0;

        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: Enter Method");
        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            Log.d("Status" , "SCHSMotor:moveStraightWithGyro: STop and Reset Loop");
            //waitOneFullHardwareCycle(); //Needed within all loops
        }
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: reset encoders");

        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: currAngle " + currAngle);

        gyroDrive(powerStart, desiredPosition, currAngle); //desired position in inches
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyroDrive finished");

        motorLeft.setPower(0);
        motorRight.setPower(0);

        double finalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: finalAngle " + finalAngle);

    }

    //aligns using gyro angles
    public void gyroDrive(double speed, double distance, double currentAngle) {
        int     newLeftTarget = 0;
        int     newRightTarget = 0;
        double  max = 0;
        double  error = 0;
        double  steer = 0;
        double  leftSpeed = 0;
        double  rightSpeed = 0;
        double countsPerInch = 0;
        double slowFactor = 0;

        Log.d("Status" , "SCHSMotor:gyroDrive: initial left position " + motorLeft.getCurrentPosition());
        Log.d("Status" , "SCHSMotor:gyroDrive: initial right position " + motorRight.getCurrentPosition());


        //converting inches to encoder values
        countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        int encoderValue = (int) (countsPerInch * distance);

        Log.d("Status" , "SCHSMotor:gyroDrive: encoder value " + encoderValue);

        double temp = countsPerInch * distance;

        // 0201 org
        // slowFactor = (Math.abs(temp) + 309)/1550;
        slowFactor = (Math.abs(temp) + 309)/1550;
        Log.d("Status" , "SCHSMotor:gyroDrive: slowFactor " + slowFactor);

        // Determine new target position, and pass to motor controller
        newLeftTarget = motorLeft.getCurrentPosition() + encoderValue;
        newRightTarget = motorRight.getCurrentPosition() + encoderValue;
        Log.d("Status" , "SCHSMotor:gyroDrive: newLeftTarget " + newLeftTarget);
        Log.d("Status" , "SCHSMotor:gyroDrive: newRightTarget " + newRightTarget);

        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        // Set Target and Turn On RUN_TO_POSITION
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), -1.0, 1.0);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
        Log.d("Status" , "SCHSMotor:gyroDrive: speed " + speed);

        double PCoeff = 0.075;
        // keep looping while we are still active, and BOTH motors are running.
        while (motorLeft.isBusy() && motorRight.isBusy()) {

            Log.d("Status" , "SCHSMotor:gyroDrive:start of loop");

            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects left" + motorLeft.getCurrentPosition());
            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects right" + motorRight.getCurrentPosition());

            // adjust relative speed based on heading error.
            error = getError(currentAngle);
            Log.d("Status" , "SCHSMotor:gyroDrive: error " + error);

            steer = getSteer(error, PCoeff);
            Log.d("Status" , "SCHSMotor:gyroDrive: steer " + steer);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0) {
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
                Log.d("Status" , "SCHSMotor:gyroDrive: speed modified slowfactor " + speed);
            }

            // if reached the desired position, exit while loop. Helps to stop turning at end of motion.
            if (distanceMovedLeft >= Math.abs(newLeftTarget) || distanceMovedRight >= Math.abs(newRightTarget)) {
                Log.d("Status" , "SCHSMotor:gyroDrive: Position reached. Break while");
                break;
            }

            Log.d("Status" , "SCHSMotor:gyroDrive:leftSpeed " + leftSpeed);
            Log.d("Status" , "SCHSMotor:gyroDrive:rightSpeed " + rightSpeed);

            motorLeft.setPower(leftSpeed);
            motorRight.setPower(rightSpeed);

            Log.d("Status", "SCHSMotor:gyroDrive:after gyro corrects left " + motorLeft.getCurrentPosition());
            Log.d("Status", "SCHSMotor:gyroDrive:after gyro corrects right " + motorRight.getCurrentPosition());
        }
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

    //turn robot to certain angle with gyro
    public void turnWithGyro(double turnSpeed , double orgTurnAngle, double direction) {

        double currGyro = 0;
        double startGyro = 0;
        double turnAngle = Math.abs(orgTurnAngle);
        Log.d("Status ", "SCHSMotorGyro:turnWithGyro: turnAngle" + turnAngle);

        double slowFactor = (0.00475 * turnAngle) + 0.505;

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        motorLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

        startGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:turnWithGyro: startGyro" + startGyro);

        currGyro = startGyro;

        //left turn
        if(direction == 1) {
            Log.d("Status" , "SCHSMotor:turnWithGyro: entered left");
            while (currGyro < startGyro + turnAngle) {
                motorLeft.setPower(-turnSpeed);
                motorRight.setPower(turnSpeed);
                currGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                Log.d("Status" , "SCHSMotor:turnWithGyro: currGyro" + currGyro);

                /*if (turnAngle < 25) { //020119 changing 0.8 to 0.9
                    if (Math.abs(currGyro - startGyro) >= 0.8 * turnAngle) {
                        turnSpeed = 0.9 * turnSpeed;
                    }
                }*/

                Log.d("Status" , "SCHSMotor:turnWithGyro: turnSpeed" + turnSpeed);

                if (currGyro >= startGyro + turnAngle) {
                    Log.d("Status ","break from turn while");
                    break;
                }

            }

        } else if (direction == 2){ //right turn
            Log.d("Status" , "SCHSMotor:turnWithGyro: entered right");
            while (currGyro > startGyro - turnAngle) {
                motorLeft.setPower(turnSpeed);
                motorRight.setPower(-turnSpeed);
                currGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                Log.d("Status" , "SCHSMotor:turnWithGyro: currGyro" + currGyro);

                /*if (turnAngle < 25) { //013119 changing 0.8 to 0.9 in if condition
                    if (Math.abs(currGyro - startGyro) >= 0.8 * turnAngle) {
                        turnSpeed = 0.9 * turnSpeed;
                    }
                }*/

                Log.d("Status" , "SCHSMotor:turnWithGyro: turnSpeed" + turnSpeed);

                if (currGyro <= startGyro - turnAngle) {
                    Log.d("Status ","break from turn while");
                    break;
                }
            }
        } else {
            Log.d("Status" , "SCHSMotor:turnWithGyro: WRONG TURN DIRECTION");
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }


}

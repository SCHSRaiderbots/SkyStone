package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSArm {

    private DcMotor liftMotor;
    private DcMotor extendMotor;

    private SCHSServo botServos;
    protected Servo grabServo;
    protected Servo leftHook;
    protected Servo rightHook;
    private int liftEncoderTarget;
    private int armEncoderTarget;
    private double liftPos;
    private double extendPos;


    public void initialize(HardwareMap hardwareMap) {
        botServos = new SCHSServo();
        botServos.initialize(hardwareMap);

        liftMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "armExtenderMotor");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        grabServo = botServos.getGrabServo();

        leftHook = botServos.getLeftHook();
        rightHook = botServos.getRightHook();
    }

    public void synchArmEncoder(int armPart) {
        //	get and set the encoder targets
        if (armPart == LIFT ){
            liftEncoderTarget = liftMotor.getCurrentPosition();
        } else if (armPart == ARM){
            armEncoderTarget = extendMotor.getCurrentPosition();
        }
    }

    public void setArmMode(DcMotor.RunMode mode, int armPart) {
        // Ensure the motors are in the correct mode.
        if (armPart == LIFT ){
            if (liftMotor.getMode() != mode)
                liftMotor.setMode(mode);
        } else if (armPart == ARM){
            if (extendMotor.getMode() != mode)
                extendMotor.setMode(mode);
        }
    }

    public void setArmPower(double power, int armPart){
        if (armPart == LIFT ){
            liftMotor.setPower(Range.clip(power, -1, 1));
        } else if (armPart == ARM){
            extendMotor.setPower(Range.clip(power, -1, 1));
        }
    }

    public void addEncoderTarget(int encoder, int armPart){
        if (armPart == LIFT ){
            liftMotor.setTargetPosition(liftEncoderTarget+= encoder);
        } else if (armPart == ARM){
            extendMotor.setTargetPosition(armEncoderTarget+= encoder);
            Log.d("SCHS:addEncoderTarget()", "armEnconderTarget = " + armEncoderTarget);
        }
    }

    public void useConstantSpeed(int armPart) {
        if (armPart == LIFT ){
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (armPart == ARM){
            extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetArmEncoders(){
        setArmPower(0,LIFT);
        setArmPower(0, ARM);
        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, LIFT);
        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, ARM);
    }

    public void openServo(Servo servo) {
        //grabServo.setDirection(Servo.Direction.FORWARD);
        Log.d("SCHS: moveServo()", "in if, current Position before" + servo.getPosition());
        //for (int i=0; i<1000; i++){
            servo.setPosition(0.99);
            Log.d("SCHS: moveServo()", "current Position after turn1:" + servo.getPosition());
            //grabServo.setPosition(0);
            //Log.d("SCHS: moveServo()", "current Position after turn2:" + grabServo.getPosition());
            //grabServo.setPosition(0.002);
            //Log.d("SCHS: moveServo()", "current Position after turn3:" + grabServo.getPosition());
        //}

    }

    public void closeServo(Servo servo) {
        //grabServo.setDirection(Servo.Direction.REVERSE);
        Log.d("SCHS: moveServo()", "in if, current Position before2:" + servo.getPosition());
        servo.setPosition(0.002);
        Log.d("SCHS: moveServo()", "current Position after turn2:" + servo.getPosition());
    }

    public void moveServoNew(double position, boolean servoDirection) {
        if (servoDirection){
            grabServo.setDirection(Servo.Direction.FORWARD);
            Log.d("Status", "moveServoNew true:set direction to forward");

            double maxPosition = position;
            double currPosition = grabServo.getPosition();
            Log.d("Status", "SCHSMotor:moveServo true: currPosition" + currPosition);

            while (currPosition <= maxPosition) {
                Log.d("Status", "SCHSMotor:moveServo true: entered while");
                maxPosition += INCREMENT;
                grabServo.setPosition(maxPosition);
                Log.d("Status", "SCHSMotor:moveServo true: max position" + maxPosition);
                sleep(CYCLE_MS);
                currPosition = grabServo.getPosition();
                Log.d("Status", "SCHSMotor:moveServo true: currPosition in loop" + currPosition);

                if (currPosition >= position){
                    Log.d("Status", "SCHSMotor:moveServo true: break");
                    break;
                }
            }

        } else {
            grabServo.setDirection(Servo.Direction.REVERSE);
            Log.d("Status", "moveServoNew false: set direction to forward");

            double minPosition = position;
            double currPosition = grabServo.getPosition();
            Log.d("Status", "SCHSMotor:moveServo false: currPosition" + currPosition);

            while (currPosition >= minPosition) {
                Log.d("Status", "SCHSMotor:moveServo false: entered while");
                minPosition += INCREMENT;
                grabServo.setPosition(minPosition);
                Log.d("Status", "SCHSMotor:moveServo false: max position" + minPosition);
                sleep(CYCLE_MS);
                currPosition = grabServo.getPosition();
                Log.d("Status", "SCHSMotor:moveServo false: currPosition in loop" + currPosition);

                if (currPosition <= position){
                    Log.d("Status", "SCHSMotor:moveServo false: break");
                    break;
                }
            }
        }
    }

    public double getLiftPos() {
        liftPos = liftMotor.getCurrentPosition();
        return liftPos;
    }

    public double getExtendPos() {
        extendPos = extendMotor.getCurrentPosition();
        return extendPos;
    }

    public int getLiftEncoderTarget() {
        return liftEncoderTarget;
    }

    public int getArmEncoderTarget() {
        return armEncoderTarget;
    }

}

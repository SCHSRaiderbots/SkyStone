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

/**
 * The class for the elevator, arm, and hooks.
 */
public class SCHSArm {
    // the lift (elevator) motor - a CoreHex
    // TODO: rename to motorLift;
    DcMotorEx liftMotor;

    // the arrm externd motor - a CoreHex
    // TODO: rename to motorExtend;
    DcMotorEx extendMotor;

    // the grabber (gripper) servo
    // TODO: rename to servoGrabber
    Servo grabServo;

    // the foundation hooks
    // TODO: make hooks private to force using the setHookState() method
    // TODO: rename to servoHookLeft and servoHookRigth
    Servo leftHook;
    Servo rightHook;

    private int liftEncoderTarget;
    private int armEncoderTarget;

    private double liftPos;
    private double extendPos;

    // TODO: rename to init()
    void initialize(HardwareMap hardwareMap) {

        // the elevator lift motor
        liftMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        // set direction should always be safe to do
        // ah, this explains why it wrapped the other way...
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // use the lift as a servo
        // set the target position to the current position.
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set power level to make it move
        liftMotor.setPower(1.0);
        // set zero power behavior (not needed with nonzero power level)
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LogDevice.dump("liftMotor", liftMotor);

        // the arm extend motor
        extendMotor = hardwareMap.get(DcMotorEx.class, "armExtenderMotor");
        // configure the motor
        // resetting the direction should always be safe
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // TODO: zero the extend motor
        //   may not want to zero extend motor at start of teleop because something might be in the way
        // We want to use the extendMotor as a servvo (run to position)
        // using DcMotor.getCurrentPosition() as initial value should always be safe
        extendMotor.setTargetPosition(extendMotor.getCurrentPosition());
        // target position must be set before RUN_TO_POSITION is invoked
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set the power to make the servo move
        extendMotor.setPower(1.0);
        // choose FLOAT or BRAKE (not needed if power is nonzero)
        extendMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        LogDevice.dump("extendMotor", extendMotor);

        // get the grabber servo
        grabServo = hardwareMap.get(Servo.class, "grabberServo");
        //grabServo.setDirection(Servo.Direction.REVERSE);
        LogDevice.dump("grabServo", grabServo);

        // get the hook servos
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        leftHook.setDirection(Servo.Direction.FORWARD);
        rightHook.setDirection(Servo.Direction.FORWARD);

        LogDevice.dump("leftHook", leftHook);
        LogDevice.dump("rightHook", rightHook);
    }

    /**
     * @deprecated just getting the armPart position will do the synch; DcMotorEx will keep track of the position.
     * @param armPart act on LIFT or ARM
     */
    public void synchArmEncoder(int armPart) {
        //	get and set the encoder targets
        if (armPart == LIFT ){
            liftEncoderTarget = liftMotor.getCurrentPosition();
        } else if (armPart == ARM){
            armEncoderTarget = extendMotor.getCurrentPosition();
        }
    }

    /**
     * @deprecated why switch on armPart?
     * @param mode desired motor mode (eg, run to position)
     * @param armPart act on LIFT or ARM
     */
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

    /**
     * @deprecated why use armPart? Why public?
     * @param power motor power to use
     * @param armPart act on LIFT or ARM
     */
    public void setArmPower(double power, int armPart){
        if (armPart == LIFT ){
            liftMotor.setPower(Range.clip(power, -1, 1));
        } else if (armPart == ARM){
            extendMotor.setPower(Range.clip(power, -1, 1));
        }
    }

    /**
     * @deprecated why use armPart? does not use standard unit of measure
     * @param encoder number of ticks to increment the encoder
     * @param armPart act on LIFT or ARM
     */
    public void addEncoderTarget(int encoder, int armPart){
        if (armPart == LIFT ){
            liftMotor.setTargetPosition(liftEncoderTarget+= encoder);
        } else if (armPart == ARM){
            //extendMotor.setTargetPosition(armEncoderTarget+= encoder);
            extendMotor.setTargetPosition(encoder);
            Log.d("SCHS:addEncoderTarget()", "encoder pos = " + encoder);
        }
    }

    /**
     * @deprecated The motors should use this run mode all the time
     * @param armPart act on LIFT or ARM
     */
    public void useConstantSpeed(int armPart) {
        if (armPart == LIFT ){
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (armPart == ARM){
            extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * @deprecated the Arm encoders should be zeroed locally
     */
    public void resetArmEncoders(){
        setArmPower(0,LIFT);
        setArmPower(0, ARM);
        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, LIFT);
        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, ARM);
    }

    /**
     * @deprecated Method should open or close the grabber rather than a generic servo
     * @param servo servo to actuate (a hook or grabber)
     */
    public void openServo(Servo servo) {
        //grabServo.setDirection(Servo.Direction.FORWARD);
        Log.d("SCHS: moveServo()", "in if, current Position before" + servo.getPosition());
        //for (int i=0; i<1000; i++){
            servo.setPosition(0.5);
            Log.d("SCHS: moveServo()", "current Position after turn1:" + servo.getPosition());
            //grabServo.setPosition(0);
            //Log.d("SCHS: moveServo()", "current Position after turn2:" + grabServo.getPosition());
            //grabServo.setPosition(0.002);
            //Log.d("SCHS: moveServo()", "current Position after turn3:" + grabServo.getPosition());
        //}
    }

    public void openGrabber() {
        Log.d("SCHS: openGrabber()", "current Position before open:" + grabServo.getPosition());
        grabServo.setPosition(Servo.MAX_POSITION);
        Log.d("SCHS: openGrabber()", "current Position after open:" + grabServo.getPosition());
    }

    public void closeGrabber() {
        Log.d("SCHS: closeGrabber()", "current Position before close:" + grabServo.getPosition());
        grabServo.setPosition(Servo.MIN_POSITION);
        Log.d("SCHS: closeGrabber()", "current Position after close:" + grabServo.getPosition());
    }

    /**
     * @deprecated SCHSArm knows which servos to use; the public should not know that detail
     * @param servo  servo to actuate (hook or grabber)
     */
    public void closeHook(Servo servo) {
        servo.setPosition(0.5); //0.1, changed to 0.4
    }

    public void openHook(Servo servo) {
        servo.setPosition(0.95);
    }

    /**
     * @deprecated The public should not have access to these servos
     * @param servo  servo to actuate (hook or grabber)
     */
    public void closeServo(Servo servo) {
        //grabServo.setDirection(Servo.Direction.REVERSE);
        Log.d("SCHS: moveServo()", "in if, current Position before2:" + servo.getPosition());
        servo.setPosition(0.002);
        Log.d("SCHS: moveServo()", "current Position after turn2:" + servo.getPosition());
    }

    /**
     * @deprecated does not use standard units
     * @return the current lift position
     */
    public double getLiftPos() {
        liftPos = liftMotor.getCurrentPosition();
        return liftPos;
    }

    /**
     * @deprecated does not use standard units
     * @return the current arm position
     */
    public double getExtendPos() {
        extendPos = extendMotor.getCurrentPosition();
        return extendPos;
    }

    /**
     * @deprecated does not use standard units; should be based on DcMotorEx.getTargetPosition()
     * @return the target lift position
     */
    public int getLiftEncoderTarget() {
        return liftEncoderTarget;
    }

    /**
     * @deprecated does not use standard units; should be based on DcMotorEx.getTargetPosition()
     * @return the target arm position
     */
    public int getArmEncoderTarget() {
        return armEncoderTarget;
    }

    /*
     * Some constants for converting lift ticks to inches
     */
    private final double LIFT_TICKS_PER_INCH = 139.77;
    private final double LIFT_INCH_OFFSET = 3.7;
    private final double LIFT_HEIGHT_MIN = 3.7;
    private final double LIFT_HEIGHT_MAX = 20.0;

    /**
     * Set the target height of lift in inches.
     * TODO: define what the height means. eg, bottom of horizontal arm.
     * @param inchHeight the target height in inches
     */
    void setLiftTargetHeight(double inchHeight) {
        // clip the height
        double inch = Range.clip(inchHeight, LIFT_HEIGHT_MIN, LIFT_HEIGHT_MAX);
        // calculate the number of ticks needed
        int ticks = (int)((inch - LIFT_INCH_OFFSET) * LIFT_TICKS_PER_INCH);

        // command the motor to that position
        liftMotor.setTargetPosition(ticks);
    }

    /**
     * Get the target height of the lift in inches
     * @return target height (inches)
     */
    double getLiftTargetHeight() {
        // get the target position in ticks
        int ticks = liftMotor.getTargetPosition();

        // convert ticks to inches
        return (ticks / LIFT_TICKS_PER_INCH) + LIFT_INCH_OFFSET;
    }

    /**
     * Get the current height of the lift in inches
     * @return current height (inches)
     */
    double getLiftCurrentHeight() {
        // get the current position in ticks
        int ticks = liftMotor.getCurrentPosition();

        // convert ticks to inches
        return (ticks / LIFT_TICKS_PER_INCH) + LIFT_INCH_OFFSET;
    }

    /**
     * Check if the lift is still moving to its commanded position
     * @return true if lift still moving
     */
    boolean isLiftBusy() {
        return liftMotor.isBusy();
    }

    /**
     * Set the lift height tolerance in inches
     * @param inchTolerance inch tolerance
     */
    void setLiftTolerance(double inchTolerance) {

        int tickTolerance = Math.max(5, (int)Math.abs(inchTolerance * LIFT_TICKS_PER_INCH));

        // set the tolerance
        liftMotor.setTargetPositionTolerance(tickTolerance);
    }

    /*
     * Some constants for converting arm ticks to inches
     */
    private final double ARM_TICKS_PER_INCH = 80.0;
    private final double ARM_INCH_OFFSET = 0.0;
    // TODO: determine arm extend limits
    private final double ARM_EXTENT_MIN = 0.0;
    private final double ARM_EXTENT_MAX = 8.0;

    /**
     * Set the target extension of the arm in inches.
     * TODO: define the extension.
     * @param inchExtend the target height in inches
     */
    void setArmTargetExtension(double inchExtend) {
        // clip the extension
        double inch = Range.clip(inchExtend, ARM_EXTENT_MIN, ARM_EXTENT_MAX);
        // calculate the number of ticks needed
        int ticks = (int)((inch - ARM_INCH_OFFSET) * ARM_TICKS_PER_INCH);

        // command the motor to that position
        extendMotor.setTargetPosition(ticks);
    }

    /**
     * Get the target extension of the arm in inches
     * @return extension in inches
     */
    double getArmTargetExtension() {
        // get the target position in ticks
        int ticks = extendMotor.getTargetPosition();

        // convert ticks to inches
        return (ticks / ARM_TICKS_PER_INCH) + ARM_INCH_OFFSET;
    }

    /**
     * Get the current extension of the arm in inches
     * @return current extension (inches)
     */
    double getArmCurrentExtension() {
        // get the current position in ticks
        int ticks = extendMotor.getCurrentPosition();

        // convert ticks to inches
        return (ticks / ARM_TICKS_PER_INCH) + ARM_INCH_OFFSET;
    }

    /**
     * Check if the lift is still moving to its commanded position
     * @return true if arm extender is still moving
     */
    boolean isArmBusy() {
        return extendMotor.isBusy();
    }

    /**
     * Set the arm extension tolerance
     * @param inchTolerance tolerance in inches
     */
    void setArmTolerance(double inchTolerance) {
        // convert inches to ticks
        int tickTolerance = Math.max(5, (int)Math.abs(inchTolerance * ARM_TICKS_PER_INCH));

        // set the tolernace
        liftMotor.setTargetPositionTolerance(tickTolerance);
    }

    /**
     * Set the foundation Hooks to a known state
     * Tte servo position values are different, so there is probably bias in servo horn.
     * The servo horn has 25 teeth, so it can only be positioned to 360/25 = 14.4 degrees
     * Set for a small change now to avoid hitting the elevator winch.
     * TODO remember the time the command was issue so completion can be estimated
     */
    void setHookState (boolean state) {
        if (state) {
            // set the hook
            leftHook.setPosition(0.5);     // larger is lower down
            rightHook.setPosition(0.55);    // smaller is lower down
        } else {
            // release the hook
            leftHook.setPosition(0.0);
            rightHook.setPosition(1.0);
        }
    }

    /**
     * Make a guess at the current Hook state by looking at left hook position.
     * The result might be wrong if the hooks were never commonanded.
     * @return true if the hooks are (probably) set
     */
    boolean getHookState () {
        return (leftHook.getPosition() > 0.25);
    }

    /**
     * Actuate the gripper
     * @param state true means gripping (closed) and falwe means released (open)
     */
    void setGrabState(boolean state) {
        if (state) {
            // if true, then we are grabbing (jaws closed
            grabServo.setPosition(0.0);
        } else {
            // if false, then we are not grabbing (jaws open)
            grabServo.setPosition(1.0);

        }
    }

    /**
     * Report the grabber's state.
     * Makes a guess taht might be wrong if the grabber has not been actuated.
     * @return true if the grabber is gripping (closed); false if released (open)
     */
    boolean getGrabState() {
        //simple comparison should give the answer
        return (grabServo.getPosition() < 0.5);
    }
}

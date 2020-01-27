package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
@Autonomous(name="SCHSController", group="SCHS")
//@Disabled
public class SCHSControllerFOUNDATION extends OpMode {

    private SCHSDrive rileyChassis = null;
    private boolean isInitialized = false;
    private SCHSDetection rileyEnv = null;
    private boolean isRoundTwo = false;
    private SCHSArm rileyArm = null;
    private int skyPos = 0;
    private double leftDist;
    private double rightDist;
    private boolean isArcTurn;
    private double blockDist = 0;
    private SCHSPathSeg[] armStonePath;
    double currArmPos;

    private enum State {
        STATE_STONES_INITIAL,
        STATE_STONES_DROP_HOOKS,
        STATE_STONES_LIFT_HOOKS,
        STATE_STONES_PULL_FD,
        STATE_STONES_PARK_BRIDGE,
        STATE_STOP
    }

    private State currState; //current state machine
    private SCHSPathSeg[] currPath; // array holding current path
    private int currSeg; //index of leg of current path

    // time into round and time into current state
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime currStateTime = new ElapsedTime();


    @Override
    public void init() {
        rileyChassis = new SCHSDrive();
        rileyChassis.initialize(hardwareMap);

        rileyEnv = new SCHSDetection();
        rileyEnv.initialize(hardwareMap);

        rileyArm = new SCHSArm();
        rileyArm.initialize(hardwareMap);

        //moved resetting encoders to init
        // TODO: Reset encoders should happen in SCHSDrive.initialize() and SCHSArm.initialize()
        rileyChassis.resetEncoders();
        rileyArm.resetArmEncoders();

        telemetry.addLine("Done Initializing");

        msStuckDetectLoop = 20000;
        msStuckDetectInit = 20000;
        msStuckDetectInitLoop = 20000;

        /* added to raise arm above block during init */
        startPath(liftArmInitial);
        //rileyArm.closeServo(rileyArm.grabServo);
        rileyArm.openGrabber();
    }

    //@Override
    public void init_loop() {
        //keep resetting encoders and show current values
        /*
        rileyChassis.resetEncoders();
        rileyArm.resetArmEncoders();
         */
        telemetry.addData("ENC", String.format("L:R %s:%s", rileyChassis.getLeftPosition(), rileyChassis.getRightPosition()));
    }

    @Override
    public void start() {
        //Set up robot devices, initial state, and game timer
        rileyChassis.setDrivePower(0,0);
        rileyArm.setArmPower(0, LIFT);
        rileyArm.setArmPower(0, ARM);
        //rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        runtime.reset();
        newState(State.STATE_STONES_INITIAL);
    }

    @Override
    public void loop() {

        switch (currState) {
            case STATE_STONES_INITIAL:
                Log.d("SCHS", "inside STATES_STONES_INITIAL");
                telemetry.addLine("inside STATES_STONES_INITIAL");
                if (rileyChassis.encodersAtZero()){
                    Log.d("SCHS", "inside STATES_STONES_INITIAL if case");
                    startPath(driveToFD);
                    newState(State.STATE_STONES_DROP_HOOKS);
                } else {
                    Log.d("SCHS", "inside STATES_STONES_INITIAL else case");
                    telemetry.addLine("inside initial else");
                }
                break;

            case STATE_STONES_DROP_HOOKS:
                if (pathComplete(DRIVE, false, false)) {
                    Log.d("SCHS", "inside STATE_STONES_DROP_HOOKS");
                    rileyArm.closeHook(rileyArm.rightHook);
                    rileyArm.openServo(rileyArm.leftHook);
                    sleep(2000);
                    newState(State.STATE_STONES_PULL_FD);
                } else {
                }
                break;

            case STATE_STONES_PULL_FD:
                Log.d("SCHS", "inside STATE_STONES_PULL_FD");
                startPath(arcTurnPushPull);
                newState(State.STATE_STONES_LIFT_HOOKS);
            break;

            case STATE_STONES_LIFT_HOOKS:
                if (pathComplete(DRIVE, false, false)) {
                    Log.d("SCHS","inside STATE_STONES_LIFT_HOOK");
                    rileyArm.openHook(rileyArm.rightHook);
                    rileyArm.closeServo(rileyArm.leftHook);
                    sleep(2000);
                    newState(State.STATE_STONES_PARK_BRIDGE);
                } else {
                }
                break;

            case STATE_STONES_PARK_BRIDGE:
                Log.d("SCHS","inside STATE_STONES_PARK_BRIDGE");
                startPath(parkUnderBridgeAfterFD);
                newState(State.STATE_STOP);
                break;

            case STATE_STOP:
                    Log.d("SCHS", "inside STATES_STOP");
                    telemetry.addLine("STATES_STOP");
                    isRoundTwo = false;
                break;
        }
    }

    @Override
    public void stop(){
        rileyChassis.useConstantSpeed();
        rileyArm.useConstantSpeed(LIFT);
        rileyArm.useConstantSpeed(ARM);
        rileyChassis.setDrivePower(0,0);
        rileyArm.setArmPower(0, LIFT);
        rileyArm.setArmPower(0, ARM);
    }

    public void newState(State newState) {
        //reset state time, change to next state
        currStateTime.reset();
        currState = newState;
    }

    public void startPath(SCHSPathSeg[] path){
        currPath = path;    // Initialize path array
        currSeg = 0;
        rileyChassis.synchEncoders(); // Lock in the current position
        rileyArm.synchArmEncoder(LIFT);
        rileyArm.synchArmEncoder(ARM);
        //rileyArm.setArmMode(DcMotor.RunMode.RUN_TO_POSITION, LIFT);
        //rileyArm.setArmMode(DcMotor.RunMode.RUN_TO_POSITION, ARM);
        startSeg();             // Execute the current (first) Leg
        //rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
    }

    public void startSeg() {
        int Left = 0;
        int Right = 0;
        int lift = 0;
        int extend = 0;

        if (currPath != null) {
            if (currPath[currSeg].isTwoSpeed == false && currPath[currSeg].armPart == 0) { //moving straight and turn in place
                // Load up the next motion based on the current segemnt.
                Left = (int) (currPath[currSeg].leftDist * COUNTS_PER_INCH);
                Right = (int) (currPath[currSeg].rightDist * COUNTS_PER_INCH);

                leftDist = Left;
                rightDist = Right;
                isArcTurn = false;

                rileyChassis.addEncoderTarget(Left, Right);
                rileyChassis.setDrivePower(currPath[currSeg].moveSpeed, currPath[currSeg].moveSpeed);
                rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
                telemetry.addLine("SCHS: startseg(): move straight/turn");
                Log.d("SCHS", "startseg(), target encoder left" + Left);
                Log.d("SCHS", "startseg(), target encoder right" + Right);
                Log.d("SCHS: startseg():", "move straight/turn");
            } else if (currPath[currSeg].armPart == LIFT) { //move lift
                lift = (int) (currPath[currSeg].liftDist * LIFT_FACTOR);
                rileyArm.addEncoderTarget(lift, LIFT);
                rileyArm.setArmPower(currPath[currSeg].liftSpeed, LIFT);
                rileyArm.setArmMode(DcMotor.RunMode.RUN_TO_POSITION, LIFT);
                telemetry.addLine("SCHS: startseg(): move lift");
                Log.d("SCHS: startseg():", "move lift");
            } else if (currPath[currSeg].armPart == ARM) { //extend arm
                telemetry.addLine("SCHS: startseg(): move arm extend before");
                Log.d("SCHS: startseg():", "move arm extend before");
                //extend = (int) (currPath[currSeg].extendDist * ARM_FACTOR);
                extend = ((int) (currPath[currSeg].extendDist * ARM_FACTOR));
                Log.d("SCHS: startseg():", "extend length (inch) = " + extend / ARM_FACTOR);
                rileyArm.addEncoderTarget(extend, ARM);
                rileyArm.setArmPower(currPath[currSeg].extendSpeed, ARM);
                rileyArm.setArmMode(DcMotor.RunMode.RUN_TO_POSITION, ARM);
                telemetry.addLine("SCHS: startseg(): move arm extend after" + rileyArm.getExtendPos());
                Log.d("SCHS: startseg():", "move arm extend after" + rileyArm.getExtendPos());
                /*} else if(currPath[currSeg].armPart == LONG_DRIVE) {
                //rileyChassis.moveStraightWithGyro(currPath[currSeg].moveSpeed, currPath[currSeg].leftDist, currPath[currSeg].rightDist);
                //replaced with DcMotorEx code

                Left = (int) (currPath[currSeg].leftDist * COUNTS_PER_INCH);
                Right = (int) (currPath[currSeg].rightDist * COUNTS_PER_INCH);

                Log.d("SCHS", "startseg(), target encoder left" + Left);
                Log.d("SCHS", "startseg(), target encoder right" + Right);

                rileyChassis.addEncoderTarget(Left, Right);
                rileyChassis.setDrivePower(currPath[currSeg].moveSpeed, currPath[currSeg].moveSpeed);
                rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
                Log.d("SCHS:", "move straight long drive");*/
            } else {//arc turn
                Left = (int) (currPath[currSeg].leftDist * COUNTS_PER_INCH);
                Right = (int) (currPath[currSeg].rightDist * COUNTS_PER_INCH);
                rileyChassis.addEncoderTarget(Left, Right);
                rileyChassis.setDrivePower(currPath[currSeg].leftSpeed, currPath[currSeg].rightSpeed);
                rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
                isArcTurn = true;
                telemetry.addLine("SCHS: startseg(): arc turn");
                Log.d("SCHS: startseg():", "arc turn");
            }
        }
        currSeg++;  // Move index to next segment of path
    }

    //checks if the current path is complete
    //As each segment completes, the next segment is started unless there are no more.
    //Returns true if the last leg has completed and the robot is stopped.

    private boolean pathComplete(int roboPart, boolean isBothArmLift, boolean isBothArmDrive) {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete(roboPart, isBothArmLift, isBothArmDrive)) {
            Log.d("SCHS: moveComplete", "moveComplete() true");

            // Start next Segement if there is one.
            if (currSeg < currPath.length)
            {
                Log.d("SCHS", "inside pathComplete() if, moveComplete = true");
                telemetry.addLine("SCHS: pathComplete(): move arm extend before startseg" + rileyArm.getExtendPos());
                Log.d("SCHS: pathComplete(): ", "move arm extend before startseg" + rileyArm.getExtendPos());
                startSeg();
                telemetry.addLine("SCHS: pathComplete(): move arm extend after startseg" + rileyArm.getExtendPos());
                Log.d("SCHS: pathComplete(): ", "move arm extend after startseg" + rileyArm.getExtendPos());

            }
            else  // Otherwise, stop and return done
            {
                Log.d("SCHS", "inside pathComplete() else, moveComplete = true");

                currPath= null;
                currSeg= 0;
                rileyChassis.setDrivePower(0, 0);
                rileyChassis.useConstantSpeed();

                //rileyArm.setArmPower(0, LIFT);
                //rileyArm.setArmPower(0, ARM);
                //rileyArm.useConstantSpeed(LIFT);
                //rileyArm.useConstantSpeed(ARM);

                return true;
            }
        }
        Log.d("SCHS: moveComplete", "moveComplete() false");
        telemetry.addLine("SCHS: startseg(): move arm extend false" + rileyArm.getExtendPos());
        return false;
    }

    // Return true if motors have both reached the desired encoder target
    public boolean moveComplete(int roboPart, boolean isBothArmLift, boolean isBothArmDrive) {
        if (isBothArmLift) {
            if (roboPart == LIFT || roboPart == ARM) {
                Log.d("SCHS:", "inside moveComplete() lift/arm");
                Log.d("SCHS:", "moveComplete() arm extend pos=" + rileyArm.getExtendPos());
                Log.d("SCHS:", "moveComplete() arm encoder target=" + rileyArm.getArmEncoderTarget());

                return ((Math.abs(rileyArm.getLiftPos() - rileyArm.getLiftEncoderTarget()) < 25) &&//10, change to 25
                        (Math.abs(rileyArm.getExtendPos() - rileyArm.getArmEncoderTarget()) < 10));//10, change to 25
            }
        } else {
            if (roboPart == DRIVE ) {
                Log.d("SCHS: moveComplete", "inside moveComplete() for DRIVE");
                Log.d("SCHS", "moveComplete(), curr target left" + rileyChassis.getLeftEncoderTarget());
                Log.d("SCHS", "moveComplete(), curr target right" + rileyChassis.getRightEncoderTarget());
                Log.d("SCHS", "moveComplete(), curr encoder left" + rileyChassis.getLeftPosition());
                Log.d("SCHS", "moveComplete(), curr encoder right" + rileyChassis.getRightPosition());
                Log.d("SCHS", "finished moveComplete():" + ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                        (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)));

                if ((leftDist > 3300 || rightDist > 3300) && !isArcTurn) {
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 60) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 60));
                } else if (((Math.abs(leftDist) < 1870) && (Math.abs(leftDist) > 1855)) || ((Math.abs(rightDist) < 1870) && (Math.abs(rightDist) > 1855))) {
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 60) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 60));
                } else{
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)); //10, change to 25
                }
            } else if (roboPart == LIFT) {
                Log.d("SCHS:", "inside moveComplete() for LIFT");
                return (Math.abs(rileyArm.getLiftPos() - rileyArm.getLiftEncoderTarget()) < 25); //10, change to 25
            } else if (roboPart == ARM) {
                Log.d("SCHS:", "inside moveComplete() for ARM");
                Log.d("SCHS:", "rileyArm getExtendPos = " +rileyArm.getExtendPos());
                Log.d("SCHS:", "rileyArm getArmEncoderTarget = " +rileyArm.getArmEncoderTarget());
                return (Math.abs(rileyArm.getExtendPos() - rileyArm.getArmEncoderTarget()) < 10);
            } else if (isBothArmDrive) {
                Log.d("SCHS:", "inside moveComplete() arm + drive");
                if(roboPart == ARM || roboPart == DRIVE) {
                    Log.d("SCHS:", "executing arm+drive check moveComplete()");
                    boolean armDone = Math.abs(rileyArm.getExtendPos() - rileyArm.getArmEncoderTarget()) < 25; //<10 change to <25
                    boolean moveDone = (Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20);
                    return (armDone && moveDone);
                }
            }
            /* } else if(roboPart == LONG_DRIVE) {
                Log.d("SCHS:", "inside moveComplete() for LONG_DRIVE");
                //return rileyChassis.isMoveDone;
                return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                        (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)); //10, change to 25
            }*/
        }
        Log.d("SCHS:", "moveComplete() no case entered");
        return false;
    }

    public void driveStraight(int leftDist, int rightDist) {
        rileyChassis.addEncoderTarget(leftDist, rightDist);
        rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
        rileyChassis.setDrivePower(currPath[currSeg].moveSpeed, currPath[currSeg].moveSpeed);

        while (rileyChassis.getRightPosition() <= (rightDist - 20) || rileyChassis.getLeftPosition() <= (leftDist - 20)) {
            double distDiff;

            //check for diff of motor distances, and apply correction to right motor
            distDiff = ((rileyChassis.getRightPosition() - rileyChassis.getLeftPosition()) * .0015);
            rileyChassis.setDrivePower(currPath[currSeg].moveSpeed + distDiff, currPath[currSeg].moveSpeed + distDiff);
        }
    }
}

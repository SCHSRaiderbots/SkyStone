package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Attempt an OpMode that runs on both 2019 and 2020 robots
 *
 * Use the RobotEx drive chassis
 */
@TeleOp(name="Test: DualBot", group="Test")
public class TeleopTest extends OpMode {

    // for Log.d() and friends, see https://developer.android.com/reference/android/util/Log.html
    //   use Log.d(TAG, <string>) to log debugging messages
    private static final String TAG = "testbot";

    // REV 2m distance sensor and attack mode
    private double distAttack = 0.0;
    private boolean bAttack = false;

    // try complicated initialization
    private int markovElevator = -1;

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    // get the robot (using extended class)
    private RobotEx robot = null;

    // start faking some action
    private enum OpCode {OC_NORMAL, OC_ATTACK, OC_HEAD, OC_LINE, OC_POINT, OC_TURN};

    private OpCode opcode = OpCode.OC_NORMAL;
    private int istate = 0;

    private double inchTargetDistance = 100.0;

    // possibly clean up the hook state after calibration
    private boolean bHookStateUnknown = true;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");

        // the robot hardware (driver, arm, ...)
        robot = new RobotEx();
        robot.init(hardwareMap, telemetry);

        // set the pose for testing
        robot.setPoseInches(0,0,0);

        // if this robot has an arm
        if (robot.arm != null) {
            // set hooks to known state
            robot.arm.setHookState(false);
        }

        // elevator initialization state
        markovElevator = -1;

        // update statistics vars
        cLoop = 0;
        timeLoop = time;

        Log.d(TAG, "init() complete");
    }

    /**
     * Quantize a number to specific steps
     * @param num the number to round
     * @param step the quantization step size
     * @return num rounded to nearest step
     */
    private double quant(double num, double step) {
        // compute the number of steps:
        long steps = Math.round(num/step);

        return step * steps;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // update drive chassis
        robot.init_loop();

        // report the identity of the robot
        telemetry.addData("Robot Identity: ", robot.robotIdentity);

        // report the distance measurement
        telemetry.addData("Distance: ", "%.2f inches", robot.inchRangeMeasurement());

        // use the left_bumper as a shift key
        if (!gamepad1.left_bumper) {
            // no left bumper modifier

            // provide instructions
            telemetry.addData("help", "X for BLUE and B for RED Alliance");
            telemetry.addData("help", "press left bumper for more");

            // Be able to set the Alliance
            //   The X button is blue alliance and the B button is red alliance
            // TODO: put alliance selection on the driver checklist
            if (gamepad1.x) {
                // BLUE Alliance
                robot.boolBlueAlliance = true;
            }
            if (gamepad1.b) {
                // RED Alliance
                robot.boolBlueAlliance = false;
            }

            // clean up after possible tests
            if (robot.arm != null) {
                // moving hooks puts them in a random state
                if (bHookStateUnknown) {
                    robot.arm.setHookState(false);
                    bHookStateUnknown = false;
                }

                if (gamepad1.y) {
                    // get the distance to target
                    double d = robot.inchRangeMeasurement();

                    // do a simple attack
                    robot.arm.setLiftTargetHeight(8.0);

                    telemetry.addData("eAttack", "Distanace %.2f Lift %.2f Arm %.2f",
                            d,
                            robot.arm.getLiftCurrentHeight(),
                            robot.arm.getArmCurrentExtension());

                    // if the distance is near and the height is safe...
                    if (d < 6.0 && robot.arm.getLiftCurrentHeight() > 6.5) {

                        // set arm just beyond
                        robot.arm.setArmTargetExtension(d + 4.5);
                    }
                }

                if (gamepad1.a) {
                    // retract the arm
                    robot.arm.setArmTargetExtension(robot.arm.ARM_EXTENT_MIN);

                    // when the arm is nearly retracted
                    if (robot.arm.getArmCurrentExtension() < robot.arm.ARM_EXTENT_MIN + 1.0) {
                        // set the lift height to 0
                        robot.arm.setLiftTargetHeight(robot.arm.LIFT_HEIGHT_MIN);
                    }
                }
            }
        } else {
            // left bumper modifier

            // This shift is used to calibrate the robot

            // if the robot has an arm,
            if (robot.arm != null) {

                // Calibrate the HOOKS

                // Hooks are in a bad state when this code exits; remember that to fix up later
                bHookStateUnknown = true;

                // provide instructions
                telemetry.addData("help", "hooks: use left and right sticks");

                // test the servos and the arm.
                // map joysticks [-1,1] to [0,1]; a 0.5 should not slam either hook...
                double hookLeft = 0.5 * (gamepad1.left_stick_y + 1.0);
                double hookRight = 0.5 * (gamepad1.right_stick_y + 1.0);

                // command the hooks -- if we have the hooks
                robot.arm.leftHook.setPosition(hookLeft);
                robot.arm.rightHook.setPosition(hookRight);

                // tell the user what the position commands are
                telemetry.addData("hooks", "left %.3f, right %.3f", hookLeft, hookRight);

                // Calibrate the ARM

                // provide instructions
                telemetry.addData("help", "lift: use left trigger; arm: use right trigger");

                // map the triggers to d4etermine the positions in inches
                double inchLift = 24.0 * (gamepad1.left_trigger);
                double inchArm = 12.0 * (gamepad1.right_trigger);

                // let's quantized the moves
                inchLift = quant(inchLift, 1.0);
                inchArm = quant(inchArm, 1.0);

                // command the arm
                robot.arm.setLiftTargetHeight(inchLift);
                robot.arm.setArmTargetExtension(inchArm);

                // get the current encoder values
                int ticksLift = (int)robot.arm.getLiftPos();
                int ticksRight = (int)robot.arm.getExtendPos();

                // report the findings
                telemetry.addData("Lift", "%.2f inch \u2192 %d ticks", inchLift, ticksLift);
                telemetry.addData("Arm", "%.2f inch \u2192 %d ticks", inchArm, ticksRight);

                // TODO Calibrate the grabber
                // SCHSArm.grabServo
            }

        }

        // Report the Alliance
        telemetry.addData("Alliance", (robot.boolBlueAlliance) ? "BLUE" : "RED");

        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Log.d(TAG, "start()");

        // pass to robot
        robot.start();

        // reset timer statistics
        cLoop = 0;
        timeLoop = time;

        // no tasks or actions
        opcode = OpCode.OC_NORMAL;
        istate = 0;

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

        // pass message to robot (will update robot position)
        robot.loop();

        // report position in meters and degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f",
                robot.xPoseInches,
                robot.yPoseInches,
                robot.thetaPoseDegrees);

        // report the distance
        telemetry.addData("Target Distance: ", "%.2f inches", inchTargetDistance);

        // report the task state
        telemetry.addData("task: ", "opcode %s, istate %d", opcode.toString(), istate);

        // dispatch on opcode
        switch (opcode) {
            case OC_NORMAL:
                // no tasks are running

                // process commands
                if (gamepad1.x) {
                    // consider an attack

                    // measure distance to possible target
                    inchTargetDistance = robot.inchRangeMeasurement();

                    // if that distance is small enough, start the attack
                    if (inchTargetDistance < 20.0) {
                        // reasonable to start an attack
                        double dist = inchTargetDistance - 5.0;

                        // move forward that amount
                        robot.execDistance(dist);

                        // set the microcontroller state
                        opcode = OpCode.OC_ATTACK;
                        istate = 0;
                    }
                } else if (gamepad1.dpad_left) {
                    // set the heading to 90
                    robot.execTurn(90.0 - robot.thetaPoseDegrees);

                    // do set heading
                    opcode = OpCode.OC_HEAD;
                } else if (gamepad1.dpad_right) {
                    // set the heading to -90
                    robot.execTurn(-90.0 - robot.thetaPoseDegrees);

                    // do set heading
                    opcode = OpCode.OC_HEAD;
                } else if (gamepad1.dpad_up) {
                    // set the heading to 0
                    robot.execTurn(0.0 - robot.thetaPoseDegrees);

                    // do set heading
                    opcode = OpCode.OC_HEAD;
                } else if (gamepad1.dpad_down) {
                    // set the heading -180
                    robot.execTurn(-180.0 - robot.thetaPoseDegrees);

                    // do set heading
                    opcode = OpCode.OC_HEAD;
                } else if (gamepad1.a) {
                    // try line following
                    robot.lfReynoldsStart();
                    opcode = OpCode.OC_LINE;
                } else{
                    // vanilla drive commands

                    // variable for each drive wheel to save power level for telemetry
                    double leftPower;
                    double rightPower;

                    // Drive should be abstracted to a class
                    //   drive commands are often nonlinear
                    //   sharing same nonlinearity across all Opmodes presents consistent interface to the driver
                    // Choose to drive using either Tank Mode, or POV Mode
                    // Comment out the method that's not used.  The default below is POV.
                    //   Uh, that's poor coding; have an attribute choose between the modes
                    //      static boolean bTankMode = false;

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

                    // use square-law drive
                    drive = drive * Math.abs(drive);
                    turn = turn * Math.abs(turn);

                    // clip ranges for good measure
                    leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                    rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

                    // Tank Mode uses one stick to control each wheel.
                    // - This requires no math, but it is hard to drive forward slowly and keep straight.
                    // leftPower  = -gamepad1.left_stick_y ;
                    // rightPower = -gamepad1.right_stick_y ;

                    // Send calculated power to wheels
                    robot.setDrivePower(leftPower, rightPower);

                    telemetry.addData("motor power", "left %.3f, right %.3f", leftPower, rightPower);
                }                break;

            case OC_ATTACK:
            case OC_HEAD:
                // Approach a Stone
                telemetry.addLine("Waiting for position to finish");

                if (!robot.motorLeft.isBusy() && !robot.motorRight.isBusy()) {
                    // motors are done moving
                    robot.setDrivePower(0.0, 0.0);

                    // restore the normal state
                    robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // remeasure the distance
                    inchTargetDistance = robot.inchRangeMeasurement();

                    // attack is over
                    opcode = OpCode.OC_NORMAL;
                }
                break;

            case OC_LINE:
                if (robot.xPoseInches > 100.0) {
                    // cutoff for testing

                    robot.lfReynoldsStop();

                    // line following is over
                    opcode = OpCode.OC_NORMAL;
                } else {
                    robot.lfReynoldsLoop();
                }
                break;

            default:
                Log.d(TAG, "opcode out of range");
                opcode = OpCode.OC_NORMAL;
                break;

        }

        if (istate < 0) {

        } else {
        }

        // Show the elapsed game time
        //   time and getRuntime() are high precision, but they are from the start of the opmode
        telemetry.addData("Status", "time: " + time);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Log.d(TAG, "stop()");

        // pass message to robot
        robot.stop();

        // TODO: it may be important to retract the hooks at stop time.
        // if we are touching the foundation, then we may void a score.

        Log.d(TAG, "stop() complete");
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SCHSDriveCarJan10", group="Iterative Opmode")

public class SCHSCarDriveJan10 extends OpMode {

    //OpMode Members

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor extenderMotor;

    private Servo leftHook;
    private Servo rightHook;
    private Servo grabberServo;  //need to find max and min positions and set accordingly

    private double turn;
    private double lPower;
    private double rPower;
    private double grabberPosition; //it is possible that the min/fully retracted position could be 0
    private double grabberPositionIncrement;
    private int towerHeight;

    private final double EXTENDER_POWER = 0.9; //arbitrary value to prevent extending too quickly
    private final double driveMultiplier = 0.5;
    private final double turnMultiplier  = 0.4;

    private double armPower;

    private int minElevatorPos = 120;
    private int minExtenderPos = 0;

    private int maxElevatorPos = 2400;
    private int maxExtenderPos = 1100;

    //false if the hooks are up, true if the hooks are down
    private boolean hooksEngaged;
    private boolean isThereBumperInput;



    //methods

    public void init() {

        turn = 0;
        grabberPosition = 0;
        towerHeight = 0;
        grabberPositionIncrement = 0.05; //testing value
        isThereBumperInput = false;

        telemetry.addData("Status:", "Initialized");

        //initialize motors

        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        extenderMotor = hardwareMap.get(DcMotor.class, "armExtenderMotor"); //temp name

        //to prevent arm from slipping and sliding
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TODO: ADD CODE TO RESET THE LEFT,  RIGHT MOTOR
        //elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //must set position before switching to RUN_TO_POSITION MODE

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);  //makes the left button take elevator down and right button take it up

        //initialize servos

        leftHook  = hardwareMap.get(Servo.class, "leftHook");  //port 0
        rightHook = hardwareMap.get(Servo.class, "rightHook"); //port 1
        grabberServo = hardwareMap.get(Servo.class, "grabberServo"); //port 2

        leftHook.setDirection(Servo.Direction.FORWARD);  //may need to switch, check during testing
        rightHook.setDirection(Servo.Direction.REVERSE);


        //code to reset hooks to up state - setting hooksEngaged to false should be done in this method
        hooksEngaged = false;

        telemetry.addData("Status:", "finished loading");

        //method to reset servo position

    }

    public void init_loop() {

        telemetry.addData("Status:", "Waiting");

        //raises the elevator slightly to allow the extender to retract fully
        if (elevatorMotor.getCurrentPosition() < minElevatorPos) {

            elevatorMotor.setTargetPosition(minElevatorPos);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(0.2);

        }

    }

    public void start(){

        runtime.reset();

    }

    public void loop(){

        /*

        CONTROLS:

        Gamepad 1:
        -Left, Right Triggers: forward and backward movement
        -x button + left stick: on the spot rotation
            -perhaps change to just left stick button + left stick y value
        -D pad up/down: keeps track of how many blocks are stacked
        -y: removes the "restrictor plate" and allows the robot to go at full speed
        -[Undecided]: Attack mode
        -[Undecided]: Incremental movement

        Gamepad 2:
        -Left, Right Triggers: move elevator up and down - need to switch to encoder position based
        -a, b: grabber servo movement (CHANGE TO A AND B WHEN LIMITS ARE FOUND)
        -Left, Right Buttons: extending arm for the servo
        -D pad up/down: controls hooks (up: hooks up, down: hooks down)

         */


        //situational tank turning with x button

        turn = gamepad1.left_stick_x * turnMultiplier;

        spotTurn();

        //default mode
        normalCar();

        hooks();


        towerHeight();
        elevatorMovement();
        extenderMovement();
        grabberServoMovement();

        update();

    }

    public void stop() {

        //also need to return extender to default lowest state

        //return elevator motor to default lowest state

        //elevatorMotor.setTargetPosition(0);
        //elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elevatorMotor.setPower(-0.2);

        /*

        while(elevatorMotor.getCurrentPosition() > 0) {

            elevatorMotor.setPower(-0.3);

        }


         */

        //return hooks to default state

        //resetHooks();


    }


    //MISC HELPERS


    private void towerHeight() {

        if (gamepad2.dpad_up)
            towerHeight++;

        if (gamepad2.dpad_down)
            towerHeight--;
    }

    //determines if there is input from the left trigger and no input from right trigger
    //for other methods to select any possible left trigger input accordingly
    private boolean leftTriggerInput(double rightInput, double leftInput) {

        //rightInput will be the right trigger/bumper, leftInput corresponds
        if(leftInput > 0 && rightInput == 0) {

            return true;

        }

        return false;
    }

    //returns true if there is input from the left button and no input from the right button
    private boolean leftBumperInput(boolean rightInput, boolean leftInput) {

        if (leftInput == true && rightInput == false) {

            return true;

        }

        return false;
    }

    //checks if there is button/bumper input
    private void isThereBumperInput() {

        if(gamepad2.left_bumper || gamepad2.right_bumper)
            isThereBumperInput = true;
        else
            isThereBumperInput = false;
    }

    //TODO: need to fix the issue in which its standing still after exceeding the range
    //temp solution: remove min encoder position
    //TODO: Convert to switch statement?
    private void powerClip(double power, int minEncoderPos, int maxEncoderPos, DcMotor desiredMotor) {

        if (desiredMotor.getCurrentPosition() < minEncoderPos) {

            if (power > 0)
                desiredMotor.setPower(power);

        }

        if (desiredMotor.getCurrentPosition() > maxEncoderPos) {

            if (power < 0)
                desiredMotor.setPower(power);

        }

        if (desiredMotor.getCurrentPosition() > minEncoderPos
                && desiredMotor.getCurrentPosition() < maxEncoderPos)
            desiredMotor.setPower(power);

    }

    //DRIVER PERIOD

    private void spotTurn(){

        if(gamepad1.left_stick_button) {

            //get power values; rPower is negative lPower so robot can turn like tank
            lPower = gamepad1.left_stick_x / 2.77;
            rPower = -lPower;

            leftMotor.setPower(lPower);
            rightMotor.setPower(rPower);

        }

    }


    private void normalCar(){

        //use triggers for power inputs

        //first condition: need to reverse - left trigger input and no right trigger input
        if(leftTriggerInput(gamepad1.right_trigger, gamepad1.left_trigger)) {

            reverse();

        } else {

            //default state - right trigger input and no left trigger input
            forward();
        }

    }

    private double restrictorPlate(double power) {

        if(!gamepad1.y) {

            power = power * driveMultiplier;

        }

        return power;
    }


    private void reverse() {

        lPower = -gamepad1.left_trigger;
        lPower = restrictorPlate(lPower);

        rPower = lPower;


        leftMotor.setPower(lPower - turn); //switch + and - if turning in wrong direction
        rightMotor.setPower(rPower + turn);

    }

    private void forward() {

        rPower = gamepad1.right_trigger;
        rPower = restrictorPlate(rPower);

        lPower = rPower;

        leftMotor.setPower(lPower + turn);
        rightMotor.setPower(rPower - turn);
    }

    //controls the servo hooks that grab the foundation
    private void hooks() {

        //fixed Jan 7
        //hooksEngaged is false, hooks are up, lower the hooks
        if (gamepad1.dpad_down && hooksEngaged == false) {

            leftHook.setPosition(0.4);
            rightHook.setPosition(0.4);

            hooksEngaged = true;
        }

        if (gamepad1.dpad_up && hooksEngaged == true) {

            leftHook.setPosition(0);
            rightHook.setPosition(0);

            hooksEngaged = false;
        }

    }


    //SERVO AND ARM MOTOR HELPER METHODS

    private String hooksInfo(){

        if (hooksEngaged == false)
            return "UP";
        if (hooksEngaged == true)
            return "DOWN";

        return("joe");
    }

    private void increment() {

    }

    //set hooks to in the up position and set hooksEngaged boolean to false
    private void resetHooks() {

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        hooksEngaged = false;

    }

    //controls the elevator - :sadjuri:
    //TODO: check if left TRIGGER makes it go down, right trigger makes it go up
    //TODO: find min/ max for range.clip
    private void elevatorMovement() {

        //OLD CODE Before Jan 10
        /*

        Probably does not work because the elevator motor mode has been changed to RUN_TO_POSITION

        if (leftTriggerInput(gamepad2.right_trigger, gamepad2.left_trigger)) {

            armPower = -gamepad2.left_trigger;
            powerClip(armPower, 18, maxElevatorPos, elevatorMotor);

        } else {

            armPower =  gamepad2.right_trigger;
            powerClip(armPower, 18, maxElevatorPos, elevatorMotor);

        }

         */

        //new code similar to last year's code

        if (leftTriggerInput(gamepad2.right_trigger, gamepad2.left_trigger))
            armPower = -gamepad2.left_trigger;
        else
            armPower =  gamepad2.right_trigger;

        int currentPosition = elevatorMotor.getCurrentPosition();
        int targetPosition  = currentPosition + (int)(armPower * 300);


        //targetPosition = Range.clip(targetPosition, minElevatorPos + 10, maxElevatorPos);

        elevatorMotor.setTargetPosition(targetPosition);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(armPower);


        //TARGET POSITION NOT SET EXCEPTION has been fixed

    }

    //extends the grabbing arm based on left/right buttons
    //left should be UP right should be DOWN

    //TODO: test, find min and max values for Range.clip
    private void extenderMovement(){

        int currentPosition = extenderMotor.getCurrentPosition();
        int targetPosition;
        double extenderPower;

        isThereBumperInput();

        //this is here so it does not extend without any bumper input - the bumpers return false compared to the triggers' 0 when not pressed
        if(isThereBumperInput) {

            if (leftBumperInput(gamepad2.right_bumper, gamepad2.left_bumper)) {

                //old code not designed for RUN_TO_POSITION
                //powerClip(-EXTENDER_POWER, 0, maxExtenderPos, extenderMotor);

                targetPosition = currentPosition + (int)(-EXTENDER_POWER * 100);
                extenderPower = -EXTENDER_POWER;

            } else {

                //if the first if condition is not satisfied then it must be the right bumper input making is there bumper input true

                //old code not designed for RUN_TO_POSITION
                //powerClip(EXTENDER_POWER, 0, maxExtenderPos, extenderMotor);

                targetPosition = currentPosition + (int)(EXTENDER_POWER * 100);
                extenderPower =  EXTENDER_POWER;

            }

            extenderMotor.setTargetPosition(targetPosition);
            extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extenderMotor.setPower(extenderPower);

        } else {

            extenderMotor.setTargetPosition(currentPosition);
            extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extenderMotor.setPower(0);
        }
    }

    //can grab blocks at driver request via button if [unmade] conditions are met
    //make this into "attack mode" method?
    private void grabBlock() {

    }


    //uses incremental thing to find limiting values
    //once these are found, TODO: make it so that the servo either goes to open position or closed position ONLY with a and b keys
    private void grabberServoMovement() {

        //retract the grabber
        if (gamepad2.dpad_down) {

            grabberPosition -= grabberPositionIncrement;

            //extend the grabber
        } else if (gamepad2.dpad_up) {

            grabberPosition += grabberPositionIncrement;

        }

        grabberServo.setPosition(grabberPosition);

    }


    //update telemetry info
    private void update() {

        telemetry.addData("Left Motor Encoder Position",  leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Encoder Position", rightMotor.getCurrentPosition());
        //telemetry.addData("Turn Multiplier value:", turnMultiplier);

        telemetry.addData(" ", " ");

        //telemetry.addData("Hook Status", hooksInfo());

        telemetry.addData("Elevator Encoder Position", elevatorMotor.getCurrentPosition());
        telemetry.addData("Extender Encoder Position", extenderMotor.getCurrentPosition());
        telemetry.addData("grabberServo position:", grabberServo.getPosition());

        telemetry.addData(" ", " ");

        telemetry.addData("towerHeight:", towerHeight);

    }

}


//add code that resets servo to "up" state
//TODO: finish the servo stuff
//TODO: find a way for the servo to return to the default retracted position
//switch arm related motors to encoder position-based movement w/ range setter [similar to last year's code]
//TODO: fix the turning
//TODO: fix the servo movement current method does  not work, see concept scan servo for incrementing


//TODO: driver period methods which move the elevator to min/max positions
/*

12/3 Testing Checklist:

1. Servos travel in same direction
2. Find Servo limits
3. Robot still drives
6. Find maximum elevator/extender encoder limits
    elevator: max is TO BE FOUND
    extender: max is TO BE FOUND

 */

//robot should keep track of how many blocks are stacked, press a button for each layer of tower


//1/8: Max extender position
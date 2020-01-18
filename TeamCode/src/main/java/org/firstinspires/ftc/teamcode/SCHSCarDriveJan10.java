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
    private double grabberPosition;
    private int towerHeight;

    private final double EXTENDER_POWER = 0.9; //arbitrary value to prevent extending too quickly
    private final double driveMultiplier = 0.5;
    private double turnMultiplier  = 0.4;

    private double armPower;

    private int minElevatorPos = -99999;
    private int minExtenderPos = -99999;

    private int maxElevatorPos = 99999;
    private int maxExtenderPos = 99999; //~2500

    private final int creepConstant = 500;

    //false if the hooks are up, true if the hooks are down
    private boolean hooksEngaged;

    private boolean isThereBumperInput;



    //methods

    public void init() {

        turn = 0;
        grabberPosition = 0;
        towerHeight = 0;
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
        //TODO: ADD CODE TO RESET THE LEFT, RIGHT MOTOR
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
        telemetry.addData("Make sure everything is reset properly!", "");

        grabberServo.setPosition(0);

        //raises the elevator slightly to allow the extender to retract fully

        /*

        if (elevatorMotor.getCurrentPosition() < minElevatorPos) {

            elevatorMotor.setTargetPosition(minElevatorPos);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(0.2);

        }

         */

    }

    public void start(){

        runtime.reset();

    }

    public void loop(){

        /*

        CONTROLS:

        Gamepad 1:
        -Left, Right Triggers: forward and backward movement
        -left stick: on the spot rotation
        -D pad up/down: controls hooks (up: hooks up, down: hooks down)
        -y: removes the "restrictor plate" and allows the robot to go at full speed

        Gamepad 2:
        -Left, Right Triggers: move elevator up and down - need to switch to encoder position based
        -x: retracts the grabber servo fully
        -b: extends the grabber servo fully
        -a: moves the robot back a small distance
        -y: moves the robot forward a small distance
        -Left, Right Bumpers: extending arm for the servo


         */


        //situational tank turning with x button


        if (gamepad1.left_stick_button)
            spotTurn(); //on the spot turning
        else if (gamepad2.a || gamepad2.y)
            creep();
        else
            normalCar();

        hooks();


        elevatorMovement();
        extenderMovement();
        grabberServoMovement();

        update();

    }

    public void stop() {

        resetHooks();


        super.stop();

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

    /*
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

     */

    //DRIVER PERIOD

    //TODO: make the robot not turn without trigger inputs, find another button for spot turning instead of left stick button
    private void setTurn() {

        turn = gamepad1.left_stick_x;

        if (gamepad1.a && gamepad1.x) {

            // a and x buttons are pressed
            //turn faster

            turnMultiplier = 0.8;

        } else {

            //default
            turnMultiplier = 0.3;

        }

        turn = turn * turnMultiplier;

    }

    private void spotTurn(){

        //get power values; rPower is negative lPower so robot can turn like tank
        lPower = gamepad1.left_stick_x;
        lPower = Math.pow(lPower, 2); //squared inputs for precise turning
        rPower = -lPower;

        leftMotor.setPower(lPower);
        rightMotor.setPower(rPower);


    }

    private void normalCar(){

        setTurn();

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            leftHook.setPosition(0.5);
            rightHook.setPosition(0.5);

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

    //TODO: MAKE similar methods to move the extender/elevator very slightly or just slow it down completely
    private void creep() {

        int currentPosLeft  = leftMotor.getCurrentPosition();
        int currentPosRight = rightMotor.getCurrentPosition();

        int targetPosLeft  = 0;
        int targetPosRight = 0;

        if (gamepad2.y) {

            targetPosLeft = currentPosLeft + creepConstant;
            targetPosRight = currentPosRight + creepConstant;

        }

        if (gamepad2.a) {

            targetPosLeft  = currentPosLeft  - creepConstant;
            targetPosRight = currentPosRight - creepConstant;

        }

        leftMotor.setTargetPosition(targetPosLeft);
        rightMotor.setTargetPosition(targetPosRight);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.1);
        rightMotor.setPower(0.1);


    }

    //set hooks to in the up position and set hooksEngaged boolean to false
    private void resetHooks() {

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        hooksEngaged = false;

    }

    //TODO: actually use this to retract the elevator or extender
    private void retract(DcMotor desiredMotor, int minPos) {

        if (desiredMotor.getCurrentPosition() > minPos) {

            desiredMotor.setTargetPosition(minPos);
            desiredMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            desiredMotor.setPower(0.5);

        }

    }

    private void retractExtender() {

        retract(extenderMotor, minExtenderPos);

    }

    private void retractElevator() {

        retract(elevatorMotor, minExtenderPos);

    }

    //controls the elevator - :sadjuri:
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


        targetPosition = Range.clip(targetPosition, minElevatorPos, maxElevatorPos);

        elevatorMotor.setTargetPosition(targetPosition);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(armPower);


        //TARGET POSITION NOT SET EXCEPTION has been fixed

    }

    //extends the grabbing arm based on left/right buttons
    //left should be UP right should be DOWN

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

            targetPosition = Range.clip(targetPosition, minExtenderPos, maxExtenderPos);

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


    //TODO in next iteration: make same key open and close the grabber
    //Perhaps make it stay open ONLY when the button is held?
    private void grabberServoMovement() {

        //retract the grabber
        if (gamepad2.x) {

            grabberPosition = 0;

            //extend the grabber
        } else if (gamepad2.b) {

            grabberPosition = 0.998;

        }

        grabberServo.setPosition(grabberPosition);

    }


    //update telemetry info
    private void update() {

        //telemetry.addData("Left Motor Encoder Position",  leftMotor.getCurrentPosition());
        //telemetry.addData("Right Motor Encoder Position", rightMotor.getCurrentPosition());
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
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Iterative Opmode")

public class ServoTest extends OpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    private double targetGrabberPosition = 0;

    private Servo leftHook;
    private Servo rightHook;
    private Servo grabberServo;

    public void init() {

        leftHook  = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        leftHook.setDirection(Servo.Direction.FORWARD);
        rightHook.setDirection(Servo.Direction.REVERSE);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberServo.setDirection(Servo.Direction.FORWARD);


    }

    public void init_loop() {


    }

    public void loop() {

        setGrabberServo();
        setHooks();
        update();

    }


    //servo returns to default state, likely because position is set to 0 each time
    public void setGrabberServo() {

        double currentPos = grabberServo.getPosition();

        if (gamepad2.a) //retract servo, eventually to open the grip
            targetGrabberPosition = currentPos + 0.05;
        else if (gamepad2.b) //extend the servo, eventually to close the grip
            targetGrabberPosition = currentPos - 0.05;

        grabberServo.setPosition(targetGrabberPosition);

    }

    public void setHooks() {

        if(gamepad2.dpad_up) {

            leftHook.setPosition(0);
            rightHook.setPosition(0);
        }

        if (gamepad2.dpad_down) {

            leftHook.setPosition(0.44);
            rightHook.setPosition(0.44);

        }

    }

    public void update() {

        telemetry.addData("leftHook Position:", leftHook.getPosition());
        telemetry.addData("rightHook Position:", rightHook.getPosition());
        telemetry.addData("grabberServo Position:", grabberServo.getPosition());

    }

    public void stop() {

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        grabberServo.setPosition(0);

    }

}

//TODO: use "scaleRange" servo method
//1414508091
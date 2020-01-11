package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SCHSServo {

    private Servo leftHook;
    private Servo rightHook;
    private Servo grabServo;

    public void initialize(HardwareMap hardwareMap) {
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        grabServo = hardwareMap.get(Servo.class, "grabberServo");

        leftHook.setDirection(Servo.Direction.FORWARD);
        rightHook.setDirection(Servo.Direction.FORWARD);
        //grabServo.setDirection(Servo.Direction.REVERSE);
    }

    public Servo getLeftHook() {
        return leftHook;
    }

    public Servo getRightHook() { return rightHook; }

    public Servo getGrabServo() {
        return grabServo;
    }
}

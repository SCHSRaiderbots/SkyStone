package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SCHSDcMotor {

    private DcMotor motorleft = null;
    private DcMotor motorRight = null;
    private DcMotor liftMotor = null;
    private DcMotor extendMotor = null;

    public void initialize(HardwareMap hardwareMap) {
        motorleft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "armExtenderMotor");

        motorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getMotorleft() {
        return motorleft;
    }

    public DcMotor getMotorRight() {
        return motorRight;
    }

    public DcMotor getLiftMotor() { return liftMotor; }

    public DcMotor getExtendMotor() { return extendMotor; }

}

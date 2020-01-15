package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SCHSDcMotor {

    private DcMotorEx motorleft = null;
    private DcMotorEx motorRight = null;
    private DcMotor liftMotor = null;
    private DcMotor extendMotor = null;

    public void initialize(HardwareMap hardwareMap) {
        motorleft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "armExtenderMotor");

        motorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotorEx getMotorleft() {
        return motorleft;
    }

    public DcMotorEx getMotorRight() {
        return motorRight;
    }

    public DcMotor getLiftMotor() { return liftMotor; }

    public DcMotor getExtendMotor() { return extendMotor; }

}

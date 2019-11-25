package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SCHSDcMotor {

    private DcMotor motorleft = null;
    private DcMotor motorRight = null;

    public void initialize(HardwareMap hardwareMap) {

        motorleft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");

        motorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public DcMotor getMotorleft() {
        return motorleft;
    }

    public DcMotor getMotorRight() {
        return motorRight;
    }

}

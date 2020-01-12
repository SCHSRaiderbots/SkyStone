package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SCHSDcMotor {

    private DcMotorEx motorleft = null;
    private DcMotorEx motorRight = null;
    private DcMotorEx liftMotor = null;
    private DcMotorEx extendMotor = null;

    public void initialize(HardwareMap hardwareMap) {
        motorleft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "armExtenderMotor");

        motorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * @Deprecated
     * @return
     */
    public DcMotorEx getMotorleft() {
        return motorleft;
    }

    /**
     * @Deprecated
     * @return
     */
    public DcMotorEx getMotorRight() {
        return motorRight;
    }

    /**
     * @Deprecated
     * @return
     */
    public DcMotorEx getLiftMotor() { return liftMotor; }

    /**
     * @Deprecated
     * @return
     */
    public DcMotorEx getExtendMotor() { return extendMotor; }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SCHSLift {

    private SCHSDcMotor armMotors;
    private DcMotor liftMotor;
    private int liftEncoderTarget;

    public void initialize(HardwareMap hardwareMap) {
        armMotors = new SCHSDcMotor();
        armMotors.initialize(hardwareMap);

        liftMotor = armMotors.getLiftMotor();
    }

    public void synchLiftEncoder() {
        //	get and set the encoder targets
        liftEncoderTarget = liftMotor.getCurrentPosition();
    }

    public void setLiftMode(DcMotor.RunMode mode) {
        // Ensure the motors are in the correct mode.
        if (liftMotor.getMode() != mode)
            liftMotor.setMode(mode);
    }

    public void setLiftPower(double power){
        liftMotor.setPower(Range.clip(power, -1, 1));
    }

    public void addEncoderTarget(int liftEncoder){
        liftMotor.setTargetPosition(liftEncoderTarget+= liftEncoder);
    }

}

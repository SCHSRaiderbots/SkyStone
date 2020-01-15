package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSPathSeg {

    protected double leftDist;
    protected double rightDist;
    protected double moveSpeed;
    protected double leftSpeed;
    protected double rightSpeed;
    protected boolean isTwoSpeed;
    protected double liftDist;
    protected double extendDist;
    protected double liftSpeed;
    protected double extendSpeed;
    protected int isArm;
    protected int armPart;

    public SCHSPathSeg(double left, double right, double speed) {
        leftDist = left;
        rightDist = right;
        moveSpeed = speed;

        leftSpeed = 0;
        rightSpeed = 0;
        isTwoSpeed = false;

        armPart = DRIVE;
        /*if (Math.abs(left) >= 48 && Math.abs(right) >= 48) {
            armPart = LONG_DRIVE;
        } else {
            armPart = DRIVE;
        }*/
    }

    public SCHSPathSeg(double left, double right, double lSpeed, double rSpeed) {
        leftDist = left;
        rightDist = right;
        moveSpeed = 0;

        leftSpeed = lSpeed;
        rightSpeed = rSpeed;
        isTwoSpeed = true;
        armPart = DRIVE;
    }

    public SCHSPathSeg(int armPartNum, double dist, double speed, String isArmNum) {
        if (armPartNum == LIFT) {
            armPart = LIFT;
            liftDist = dist;
            liftSpeed = speed;
        } else if (armPartNum == ARM) {
            armPart = ARM;
            extendDist = dist;
            extendSpeed = speed;
        }
        isArm = 1;
    }
}
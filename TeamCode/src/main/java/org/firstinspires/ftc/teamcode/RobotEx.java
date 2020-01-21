package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Work around to avoid editing SCHSDrive and resulting git havoc
 *
 * The superclass has
 *   the drive motors
 *   the gyro sensor (this sensor is a hazard because it may cause a reboot)
 *
 * Extend the class to include a REV 2 meter distance sensor
 */
public class RobotEx extends SCHSDrive {
    // Remember some useful class members
    private Telemetry telemetry = null;

    // Extend with the arm...
    SCHSArm arm = null;

    // the 2m distance sensor
    // access is private so nobody else gets their hands on it
    private DistanceSensor sensorRange2m = null;

    // report the distance from the front of the chassis
    // this value adjusts for the placement and offset of the sensor
    private double inchDistanceOffset = 0.0;

    // the touch sensor is used to discriminate robots
    //   belief is the sensor will exist on the 2019 robot but not the 2020 robot
    private DigitalChannel digitalTouch = null;
    String robotIdentity = "unknown";

    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    // which Alliance?
    boolean boolBlueAlliance = true;

    public void init(HardwareMap hwmap, Telemetry telem) {
        // call the superclass to init its items
        super.init(hwmap, telem);

        // TODO: do not access motors directly; use methods
        motorLeft = super.motorLeft;
        motorRight = super.motorRight;

        // remember the telemetry class member so it can be used in other methods
        telemetry = telem;

        // find the REV Robotics 2m distance sensor
        // TODO: or more exotic sensor; this may be the source of the I2C 0x52 error message
        // can use as an ordinary distance sensor
        sensorRange2m = hardwareMap.get(DistanceSensor.class, "rev2meter");
        // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange2m;

        // try to discriminate the two robots by looking for a particular sensor
        // touch sensor
        digitalTouch = hardwareMap.tryGet(DigitalChannel.class, "digitalTouchRiley");
        if (digitalTouch == null) {
            // the sensor was not found, so presume it is the 2020 robot
            robotIdentity = "2020 Robot";

            // get the arm
            arm = new SCHSArm();

            // and initialize it
            arm.initialize(hardwareMap);
        } else {
            // the sensor was not found, so presume it is the 2019 robot
            robotIdentity = "2019 Robot";

            // adjust the robot parameters
            setRobot2019();
        }
    }

    public void init_loop() {
        // call the superclass
        super.init_loop();
    }

    public void start() {
        // call the superclass
        super.start();
    }

    public void loop() {
        // call the superclass
        super.loop();
    }

    public void stop() {
        // call the superclass
        super.stop();
    }

    /**
     * Often want to set both left and right motors to the same mode
     * @param mode DcMotor.RunMode for the drive motors
     */
    void setDriveMode(DcMotor.RunMode mode) {
        motorLeft.setMode(mode);
        motorRight.setMode(mode);
    }

    /**
     * Get a range measurement using the REV 2m Distance sensor, which is an infrared time-of-flight.
     * The sensor takes about 20 ms to perform the measurement, so do not do it all the time.
     * This method adds inchDistanceOffset to adjust for the location of the sensor on the robot.
     *
     * @return distance from front of chassis to target
     */
    double inchRangeMeasurement() {
        // read the distance sensor
        // this operation is SLOW! it takes about 20 ms.
        double inchDistance = sensorRange2m.getDistance(DistanceUnit.INCH);

        // adjust for position and reading errors
        return inchDistance + inchDistanceOffset;
    }

    void execDistance(double inchLeft, double inchRight) {
        // compute number of ticks
        int ticksLeft = ticksFromInches(inchLeft);
        int ticksRight = ticksFromInches(inchRight);

        // set the target position
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + ticksLeft);
        motorRight.setTargetPosition(motorRight.getCurrentPosition() + ticksRight);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(1.0, 1.0);
    }

    void execDistance(double inchForward) {
        execDistance(inchForward, inchForward);
    }

    void execTurn(double degTurn) {
        // normalize the angle
        while (degTurn > 180.0) {
            degTurn -= 360.0;
        }
        while (degTurn < -180.0) {
            degTurn += 360.0;
        }

        // convert to radians
        double radians = degTurn * Math.PI / 180.0;
        // calculate distance in meters
        double mDist = radians * distWheel;
        // convert to ticks
        int ticks = ticksFromMeters(mDist);

        // set the target position
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition() - ticks);
        motorRight.setTargetPosition(motorRight.getCurrentPosition() + ticks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(0.5, 0.5);
    }


    // Craig W. Reynolds ad hoc line follower (from memory)
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 120.0;
    double y1 = 0.0;

    void lfReynoldsStart() {
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition());
        motorRight.setTargetPosition(motorRight.getCurrentPosition());

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 0.50 seems to work
        motorLeft.setPower(1.0);
        motorRight.setPower(1.0);
    }

    void lfReynoldsLoop() {
        // compute vector from starting position (x0, y0)
        double dx = x1 - x0;
        double dy = y1 - y0;

        // make a unit vector
        double L = Math.hypot(dx, dy);
        double ux = dx/L;
        double uy = dy/L;

        // compute vector to robot
        double dxBot = xPoseInches - x0;
        double dyBot = yPoseInches - y0;

        // Log.d("CWR", "Pose " + xPoseInches + " " + yPoseInches);

        // compute the dot product to get the length along the line
        double dot = ux * dxBot + uy * dyBot;

        // if dot > L, we are off the target

        // make a guess at the desired dL for bot to progress
        double dL = 12.0;

        // if dot + dL > L, we are off the target

        // here is an aiming point on the path
        double x2 = x0 + (dot + dL) * ux;
        double y2 = y0 + (dot + dL) * uy;

        // Log.d("CWR", "Aiming point " + x2 + " " + y2);

        // the bot is headed thetaPose
        // the aiming correction is
        double thetaAim = Math.atan2(y2-yPoseInches, x2-xPoseInches) - thetaPose;

        // reduce the angle
        while (thetaAim > Math.PI) thetaAim -= 2 * Math.PI;
        while (thetaAim < -Math.PI) thetaAim += 2 * Math.PI;

        // Log.d("CWR", "thetaAim = " + thetaAim);

        // over the distance L, want to correct by some fraction of that angle
        double wb = 9.0;
        double distCorr = wb * thetaAim;

        // Log.d("CWR", "distCorr = " + distCorr);

        // motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + ticksFromInches(L-distCorr));
        // motorRight.setTargetPosition(motorRight.getCurrentPosition() + ticksFromInches(L+distCorr));

        // TODO this correction seems huge
        //  100 has overshoot
        //  200 seems OK
        //  400 also works
        double f = thetaAim * 400;

        motorLeft.setVelocity(400-f);
        motorRight.setVelocity(400+f);
    }

    void lfReynoldsStop() {
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition());
        motorRight.setTargetPosition(motorRight.getCurrentPosition());

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setPower(1.0);
        motorRight.setPower(1.0);
    }

}

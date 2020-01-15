package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Forward 24 inches - Riley", group="Test")
public class AutoForward24 extends OpMode {
    // Declare OpMode members.

    // for Log.d() and friends, see https://developer.android.com/reference/android/util/Log.html
    private static final String TAG = "testbot";
    // so use Log.d(TAG, <string>) to log debugging messages

    // drive motors
    // abstract to a class (eg, Robot) where attributes can be static and shared by other Opmodes
    //   the class can have methods such as .setDrivePower()
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    // SCHSDrive (has drive motors and other stuff)
    private SCHSDrive schsdrive = null;

    private int iState = 0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");
        telemetry.addData("Status", "Initializing");

        // the robot drive
        schsdrive = new SCHSDrive();
        schsdrive.init(hardwareMap, telemetry);
        // use 2019 values
        schsdrive.setRobot2019();

        // set the pose for testing
        schsdrive.setPoseInches(0,0,0);

        leftDrive = schsdrive.motorLeft;
        rightDrive = schsdrive.motorRight;

        // update statistics vars
        cLoop = 0;
        timeLoop = time;

        // Tell the driver station that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Log.d(TAG, "init() complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("init", "looping; look for config info");

        // update drive chassis
        schsdrive.init_loop();

        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Log.d(TAG, "start()");

        schsdrive.start();

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);

        // reset timer statistics
        cLoop = 0;
        timeLoop = time;

        Log.d(TAG, "start() complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int ticks = schsdrive.ticksFromInches(24.0);

        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

        // update robot position
        schsdrive.loop();

        // report position in meters and degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f",
                schsdrive.xPoseInches,
                schsdrive.yPoseInches,
                schsdrive.thetaPoseDegrees);

        telemetry.addData("velocity", "%.3f %.3f ticks/second",
                leftDrive.getVelocity(),
                rightDrive.getVelocity());

        telemetry.addData("state", " %3d", iState);

        switch (iState) {
            case 0:
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);
                iState++;
                break;

            case 1:
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    telemetry.addLine("forward finished");

                    // ticks = schsdrive.ticksFromMeters(0.5 * Math.PI * schsdrive.distWheel);

                    // turn around
                    // leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
                    // rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);
                    iState++;
                }
                break;

            case 2:
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    telemetry.addLine("turn finished");

                    ticks=0;

                    // drive back
                    leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - ticks);
                    rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);
                    iState++;
                }
                break;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + time);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Log.d(TAG, "stop()");

        schsdrive.stop();

        Log.d(TAG, "stop() complete");
    }
}

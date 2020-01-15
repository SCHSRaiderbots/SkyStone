package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Forward 24 inches", group="Test")
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

        // set the pose for testing
        schsdrive.setPoseInches(0,0,0);

        // TODO stop using local copies
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
        // TODO why is this loop taking 100 ms?
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

        int ticks = schsdrive.ticksFromInches(72.0);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

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

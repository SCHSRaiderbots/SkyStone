package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test: Forward 24 inches", group="Test")
public class AutoForward24 extends OpMode {
    // Declare OpMode members.

    // for Log.d() and friends, see https://developer.android.com/reference/android/util/Log.html
    // so use Log.d(TAG, <string>) to log debugging messages
    private static final String TAG = "Forward 24 inches";

    // average period statistics
    private int cLoop = 0;
    private double timeLoop = 0;

    // SCHSDrive (has drive motors and other stuff)
    private RobotEx robot = null;

    private int iState = 0;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Log.d(TAG, "init()");
        telemetry.addData("Status", "Initializing");

        // the robot drive
        robot = new RobotEx();
        robot.init(hardwareMap, telemetry);

        // set the pose for testing
        robot.setPoseInches(0,0,0);

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
        robot.init_loop();

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

        robot.start();

        robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition());
        robot.motorRight.setTargetPosition(robot.motorRight.getCurrentPosition());

        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDrivePower(0.5, 0.5);

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
        int ticks = robot.ticksFromInches(24.0);

        // update statistics for loop period
        cLoop++;
        telemetry.addData("average period", "%.3f ms", 1000*(time-timeLoop) / cLoop);

        // update robot position
        robot.loop();

        // report position in meters and degrees
        telemetry.addData("pose", "%8.2f %8.2f %8.2f",
                robot.xPoseInches,
                robot.yPoseInches,
                robot.thetaPoseDegrees);

        telemetry.addData("velocity", "%.3f %.3f ticks/second",
                robot.motorLeft.getVelocity(),
                robot.motorRight.getVelocity());

        telemetry.addData("state", " %3d", iState);

        switch (iState) {
            case 0:
                robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition() + ticks);
                robot.motorRight.setTargetPosition(robot.motorRight.getCurrentPosition() + ticks);
                iState++;
                break;

            case 1:
                if (!robot.motorLeft.isBusy() && !robot.motorRight.isBusy()) {
                    telemetry.addLine("forward finished");

                    // ticks = schsdrive.ticksFromMeters(0.5 * Math.PI * schsdrive.distWheel);

                    // turn around
                    // leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
                    // rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);
                    iState++;
                }
                break;

            case 2:
                if (!robot.motorLeft.isBusy() && !robot.motorRight.isBusy()) {
                    telemetry.addLine("turn finished");

                    ticks=0;

                    // drive back
                    robot.motorLeft.setTargetPosition(robot.motorLeft.getCurrentPosition() - ticks);
                    robot.motorRight.setTargetPosition(robot.motorRight.getCurrentPosition() - ticks);
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

        robot.stop();

        Log.d(TAG, "stop() complete");
    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

@Autonomous(name="SCHSController", group="SCHS")
//@Disabled
public class SCHSController extends LinearOpMode {

    private Raiderbot riley = null;
    private boolean isInitialized = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void initialize() {
        riley = new Raiderbot();

        Log.d("Status" , "Controller:initialize: before riley initialized");

        riley.initialize(hardwareMap);
        isInitialized = true;

    }

    public void cleanShutDown() {
        riley.cleanShutDown();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        if (isInitialized != true) {
            initialize();
            Log.d("Status" , "SCHSController:runOpMode: inside initialized runopmode");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Log.d("Status" , "SCHSController:runOpMode: program started");

        riley.testFunctions();

        cleanShutDown();

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //}
    }
}

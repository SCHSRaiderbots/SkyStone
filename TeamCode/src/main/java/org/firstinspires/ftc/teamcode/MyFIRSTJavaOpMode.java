package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Recreated by GLR 8 October 2019
 * TODO: Move code to common class (eg, Robot) and specific Opmodes; Lousy name for class
 */
@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorLeft;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorRange2m;
    private ColorSensor sensorColor;
    private Servo servoTest;

    @Override
    public void runOpMode() {
        // these throw errors if the device does not exist
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        // can use as an ordinary distance sensor
        sensorRange2m = hardwareMap.get(DistanceSensor.class, "rev2meter");
        // or cast as more exotic sensor
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange2m;

        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // HSV values
        float hsvValues[] = {0F, 0F, 0F};

        // scale values
        final double SCALE_FACTOR = 255;

        // set digital channel to input
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // issue a telemetry report
        telemetry.addData("Status", "initialized");
        telemetry.addData("sensorColor", sensorColor.getDeviceName());
        telemetry.addData("sensorDistance", sensorDistance.getDeviceName());
        telemetry.addData("sensor2m", sensorRange2m.getDeviceName());
        telemetry.update();

        // wait for start
        waitForStart();

        // run until stopped
        while (opModeIsActive()) {
            telemetry.addData("Status", "running");

            // touch sensor status
            telemetry.addData("Touch", (digitalTouch.getState()) ? "no" : "yes");

            // measure distance in cm
            telemetry.addData("distance",
                    String.format(Locale.US, "%.02f cm",
                            sensorDistance.getDistance(DistanceUnit.CM)));

            // try the 2m sensor
            double tau = getRuntime();
            double d = sensorRange2m.getDistance(DistanceUnit.CM);
            // raw time is about 17  to 20 ms
            tau = getRuntime() - tau;

            telemetry.addData("distance2m",
                    String.format(Locale.US, "%8.02f cm, % 8.02f ms", d, tau * 1000)
                    );
            // Rev2mDistanceSensor specific methods.
            // this reports "ee"
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            // this is always "false" (even with distance = 819cm (8 meters out)
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            // color sensor status
            // color sensor is Rev v 3
            // white point 2cm away 4640 8563 4095
            // black point           125 238 105

            // had used graph to scale red / 0.86, green / 0.98, and blue / 0.61
            // get values
            double tColor = getRuntime();
            int rColor = sensorColor.red();
            int gColor = sensorColor.green();
            int bColor = sensorColor.blue();
            int aColor = sensorColor.alpha();

            // time to get values is about 50 microseconds
            tColor = getRuntime() - tColor;

            telemetry.addData("raw RGB",
                    String.format(Locale.US, "%d %d %d / %d in %8.02f ms",
                            rColor, gColor, bColor, aColor,
                            tColor));

            int rCC = (int) ((rColor-125) * SCALE_FACTOR * (8563.0/4640.0));
            int gCC = (int) ((gColor-238) * SCALE_FACTOR);
            int bCC = (int) ((bColor-105) * SCALE_FACTOR * (8563.0/4095.0));

            rCC = Math.max(rCC, 0);
            gCC = Math.max(gCC, 0);
            bCC = Math.max(bCC, 0);

            // convert to HSV
            // yellow should be H = 60; I'm getting 40
            // stone: 40 (yellow should be 60 .. halfway to green at 120)
            // red gaffer tape: 12 (red should be 0; that red is decidely orange)
            // blue gaffer tape: 230 (blue should be 240)
            Color.RGBToHSV(rCC, gCC, bCC, hsvValues);
            telemetry.addData("HSV",
                    String.format(Locale.US, "%6.1f %6.3f %6.3f",
                            hsvValues[0],
                            hsvValues[1],
                            hsvValues[2]));

            telemetry.update();
        }
    }
}

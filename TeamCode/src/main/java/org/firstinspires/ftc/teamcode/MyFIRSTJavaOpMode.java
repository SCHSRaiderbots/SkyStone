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
        telemetry.addData("radioBuild", "true dat now");
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
            telemetry.addData("distance2m",
                    String.format(Locale.US, "%8.02f cm",
                            sensorRange2m.getDistance(DistanceUnit.CM))
                    );
            // Rev2mDistanceSensor specific methods.
            // this reports "ee"
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            // this is always "false" (even with distance = 819cm (8 meters out)
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            // color sensor status
            telemetry.addData("RGB",
                    String.format(Locale.US, "%d %d %d / %d",
                            sensorColor.red(),
                            sensorColor.green(),
                            sensorColor.blue(),
                            sensorColor.alpha()));
            Color.RGBToHSV(
                    (int) (sensorColor.red() * SCALE_FACTOR / 0.86),
                    (int) (sensorColor.green() * SCALE_FACTOR / 0.95),
                    (int) (sensorColor.blue() * SCALE_FACTOR / 0.61),
                    hsvValues);
            telemetry.addData("HSV",
                    String.format(Locale.US, "%6.1f %6.3f %6.3f",
                            hsvValues[0],
                            hsvValues[1],
                            hsvValues[2]));

            telemetry.update();
        }
    }
}

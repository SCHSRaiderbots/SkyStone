package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDetection {
    protected int skyPos;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     *
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private VuforiaLocalizer.Parameters parameters;

    private DistanceSensor distSensor;

    public SCHSDetection() {
        skyPos = 0;
    }

    public void initialize(HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            Log.d("SCHS: Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        distSensor = hardwareMap.get(DistanceSensor.class, "rev2meter");
    }

    public int detectSkyPos(){
        long startTime = System.currentTimeMillis();
        int skyPos = 0;
        double leftVal;
        double rightVal;
        boolean isDone = false;

        while((System.currentTimeMillis() - startTime)<= SCAN_BALLS_TIME) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                Log.d("Status", "SCHSObjectDetection: # Object Detected " + updatedRecognitions.size());
                Log.d("Status", "SCHSObjectDetection: TIME: " + (System.currentTimeMillis()-startTime));

                if (updatedRecognitions.size() >= 1 && updatedRecognitions.size() <= 3) {
                    for (Recognition recognition : updatedRecognitions) {
                        Log.d("SCHS:Tensor", "stone label:" + recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            Log.d("SCHS:Tensor", "detected Skystone");
                            leftVal = recognition.getLeft();
                            rightVal = recognition.getRight();
                            if (/*leftVal > POS_3_LEFT_MIN && leftVal < POS_3_LEFT_MAX && */ rightVal > POS_3_RIGHT_MIN && rightVal < POS_3_RIGHT_MAX) {
                                skyPos = 3;
                                Log.d("SCHS:Tensor","skyPos 3 leftVal:" + leftVal);
                                Log.d("SCHS:Tensor","skyPos 3 rightVal:" + rightVal);
                                Log.d("SCHS:Tensor","skyPos :" + skyPos);
                                isDone = true;
                                break;
                            } else if (/*leftVal > POS_2_LEFT_MIN && leftVal < POS_2_LEFT_MAX && */ rightVal > POS_2_RIGHT_MIN && rightVal < POS_2_RIGHT_MAX) {
                                skyPos = 2;
                                Log.d("SCHS:Tensor","skyPos 2 leftVal:" + leftVal);
                                Log.d("SCHS:Tensor","skyPos 2 rightVal:" + rightVal);
                                Log.d("SCHS:Tensor","skyPos :" + skyPos);
                                isDone = true;
                                break;
                            } else if (/*leftVal > POS_1_LEFT_MIN && leftVal < POS_1_LEFT_MAX && */ rightVal > POS_1_RIGHT_MIN && rightVal < POS_1_RIGHT_MAX) {
                                skyPos = 1;
                                Log.d("SCHS:Tensor","skyPos 1 leftVal:" + leftVal);
                                Log.d("SCHS:Tensor","skyPos 1 rightVal:" + rightVal);
                                Log.d("SCHS:Tensor","skyPos :" + skyPos);
                                isDone = true;
                                break;
                            } else {
                                skyPos = 2;
                                Log.d("SCHS:Tensor","Wrong left val/right val, default to MB route");
                                Log.d("SCHS:Tensor","leftVal:" + leftVal);
                                Log.d("SCHS:Tensor","rightVal:" + rightVal);
                                Log.d("SCHS:Tensor","skyPos :" + skyPos);
                                isDone = false;
                            }
                        }//end of if
                    }//end of for
                }//end of if
            }//end of if
            if (isDone){
                break;
            }
        }//end of while
        return skyPos;
    }//end of detectSkyPos()

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public double findDist() {
        return (distSensor.getDistance(DistanceUnit.INCH));
    }
}

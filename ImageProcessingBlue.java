package org.firstinspires.ftc.teamcode;

import android.nfc.Tag;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;

public class ImageProcessingBlue {

    private String TAG = "ImageProcessing";

    private static final String VUFORIA_KEY =
            "AQSKeyz/////AAABmRytc1C8SUgbsE1v0IwnpamOCO0QAw37Ibk6D0UwFr1G5yEjrfFTYrrP9euDdmXfTLpnMlf6kAhcBrs/T3KYkirjTa6Td1Q47QzqDjYr4FXN2FPp9GpfWuVIK4QCBWMGPRHJbu06/iBkcrFyRWk/ZNkw0D2mf0UC2KwUnJzR9UmwI+LAuwm2UdLIj5nsKkzObmqfQK7Fipjrn/zrD6nlkExWd0nTnQXMnR5aAhK73Nj48+WCqw7+p3jtkxbPrQLUC6Jex0HSQxbGMtz2HXgrCGvnj/vQHuRXO7v/kNgJnOzqfP2y2B9KOfnifffudXxucI4+JH8KhB35O6J5D1fnc4XGwf048/UmXn2Ax40O/iqY";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private float left;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public enum DuckPositionBlue
    {
        LEFT, CENTER, RIGHT,
    }

    public void InitImageProcessing(HardwareMap hardwareMap) {
        Log.d(TAG, "Start init");
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.2, 16.0 / 9.0);
        }
         Log.d(TAG, "Finish init");

    }

    public DuckPositionBlue FindDuckPosition()
    {
        DuckPositionBlue duckPosition = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                Log.d("Autonomous", "# O" +
                        "" +
                        "bject Detected" + updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                        /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());*/
                    Log.d(TAG, "label " + i + "," + recognition.getLabel());
                    Log.d(TAG, " left,top " + i + "," +
                            recognition.getLeft() + "," + recognition.getTop());
                    Log.d(TAG, "  right,bottom " + i + "," +
                            recognition.getRight() + "," +recognition.getBottom());
                    i++;
                    left = recognition.getLeft();
                }

                if (i == 0)
                {
                    Log.d(TAG, "The Duck in left");
                    duckPosition = DuckPositionBlue.LEFT;
                }
                else if (i == 1)
                {
                    if (left < 100)
                    {
                        Log.d(TAG, "The Duck in the center");
                        duckPosition = DuckPositionBlue.CENTER;
                    }
                    else if (left > 400)
                    {
                        Log.d(TAG, "The Duck in right");
                        duckPosition = DuckPositionBlue.RIGHT;
                    }
                    else
                    {
                        Log.d(TAG, "The Duck position is not recognize");
                    }
                }
                else
                {
                    Log.d(TAG, "Find more then one Duck");
                }
            }
        }
        return (duckPosition);
    }

    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}

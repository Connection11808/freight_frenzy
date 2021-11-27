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

@Autonomous(name="AutonomousConnection", group="Pushbot")
public class AutonomousConnection extends LinearOpMode {

    ImageProcessing imageProcessing = new ImageProcessing();
    ConnectionHardware robot       = new ConnectionHardware();
    ImageProcessing.DuckPosition duckPosition;
    private String TAG = "AutonomousConnection";

    @Override
    public void runOpMode() {
        telemetry.addLine("Start init");
        imageProcessing.InitImageProcessing(hardwareMap);
        telemetry.addLine("Finish init");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        duckPosition = imageProcessing.FindDuckPosition();
        Log.d(TAG, "DuckPosition = " + duckPosition);
        if (duckPosition == ImageProcessing.DuckPosition.LEFT)
        {
            telemetry.addLine("The Duck Position is Left");
        }
        if (duckPosition == ImageProcessing.DuckPosition.CENTER)
        {
            telemetry.addLine("The Duck Position is Center");
        }
        if (duckPosition == ImageProcessing.DuckPosition.RIGHT)
        {
            telemetry.addLine("The Duck Position is Right");
        }
        telemetry.update();
        while (opModeIsActive());

    }

}

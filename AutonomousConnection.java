package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PushbotAutoDriveByEncoder_Linear_Connection.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.PushbotAutoDriveByEncoder_Linear_Connection.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.PushbotAutoDriveByGyro_Linear_Connection.HEADING_THRESHOLD;

import android.nfc.Tag;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ConnectionHardware;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ConnectionHardware;
import org.firstinspires.ftc.teamcode.ImageProcessing;
import org.firstinspires.ftc.teamcode.PushbotAutoDriveByEncoder_Linear_Connection;

import java.util.*;

@Autonomous(name="AutonomousConnection", group="Pushbot")
public class AutonomousConnection extends LinearOpMode {

    ImageProcessing imageProcessing = new ImageProcessing();
    PushbotAutoDriveByEncoder_Linear_Connection pushbotAutoDriveByEncoderLinearConnection = new PushbotAutoDriveByEncoder_Linear_Connection();
    ConnectionHardware robot       = new ConnectionHardware();
    private ElapsedTime     runtime = new ElapsedTime();
    ImageProcessing.DuckPosition duckPosition;
    private String TAG = "AutonomousConnection";
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {
        telemetry.addLine("Start init");
        imageProcessing.InitImageProcessing(hardwareMap);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

    robot.init(hardwareMap);


    // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
    //robot.leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //robot.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //robot.leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //robot.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    telemetry.addData(">", "Robot Ready.");    //
    telemetry.update();

    waitForStart();
        duckPosition = imageProcessing.FindDuckPosition();
        Log.d(TAG, "DuckPosition = " + duckPosition);
        /*if (duckPosition == ImageProcessing.DuckPosition.LEFT)
        {
            telemetry.addLine("The Duck Position is Left");
            Log.d(TAG, "The Duck Position is Left");
            gyroDrive(0.8, 30, 0);
            gyroTurn(0.6,-90);
        }

        if (duckPosition == ImageProcessing.DuckPosition.CENTER)
        {
            telemetry.addLine("The Duck Position is Center");
            Log.d(TAG, "The Duck Position is Center");
            gyroDrive(0.8, 30, 0);
            gyroTurn(0.6,-90);
        }
        if (duckPosition == ImageProcessing.DuckPosition.RIGHT)
        {
            telemetry.addLine("The Duck Position is Right");
            Log.d(TAG, "The Duck Position is Right");
            gyroDrive(0.8, 30, 0);
            gyroTurn(0.6,-90);
        }*/

        gyroDrive(0.8, 30, 0);
        gyroTurn(0.6,-90);
        gyroDrive(0.8, 50, -90);
        gyroTurn(0.6,0);
        gyroDrive(0.8, 10, 0);
        sleep(1000);
        gyroDrive(-0.8, 10, 0);

        telemetry.update();
        /*gyroTurn(0.8, 90);
        sleep(1000);
        gyroTurn(0.8, 45);
        sleep(1000);
        gyroTurn(0.8, 22.5);*/

            /*gyroDrive(0.8, 100, 0);
            gyroTurn(0.6, 90);
            gyroDrive(0.8, 50, 90);
            gyroTurn(0.6, 180);
            gyroDrive(0.8, 100, 180);
            gyroTurn(0.6, 270);
            gyroDrive(0.8, 50, 270);
            gyroTurn(0.6, 360);*/

        Log.d(TAG, "rightDriveF Motor position is =" + robot.rightDriveF.getCurrentPosition());
        Log.d(TAG, "leftDriveF Motor position is =" + robot.leftDriveF.getCurrentPosition());
        Log.d(TAG, "rightDriveB Motor position is =" + robot.rightDriveB.getCurrentPosition());
        Log.d(TAG, "leftDriveB Motor position is =" + robot.leftDriveB.getCurrentPosition());
        //while (opModeIsActive());

    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Updates telemetry & Allow time for other processes to run.
            telemetry.update();
        }

    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDriveF.setPower(leftSpeed);
        robot.rightDriveF.setPower(rightSpeed);
        robot.leftDriveB.setPower(leftSpeed);
        robot.rightDriveB.setPower(rightSpeed);

        // Display it for the driver.
        /*telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);*/

        Log.d(TAG, "Target" + " " + angle);
        Log.d(TAG, "Err/St" + " " + error + " " + steer);
        Log.d(TAG, "Speed" + " " + leftSpeed + " " + rightSpeed);
        Log.d(TAG, "" + robot.GetImuAngle());


        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetImuAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFTarget;
        int     newLeftBTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int positionLeftDriveF;
        int positionLeftDriveB;
        int rangLeftDriveF;
        int rangLeftDriveB;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

             distance = robot.cm_to_inch(distance);
             Log.d(TAG, "the distance is (inch) = " + distance);
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFTarget = robot.leftDriveF.getCurrentPosition() + moveCounts;
            //newRightTarget = robot.rightDriveF.getCurrentPosition() + moveCounts;
            newLeftBTarget = robot.leftDriveB.getCurrentPosition() + moveCounts;
            //newRightTarget = robot.rightDriveB.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftDriveF.setTargetPosition(newLeftFTarget);
            //robot.rightDriveF.setTargetPosition(newRightTarget);
            //robot.leftDriveB.setTargetPosition(newLeftBTarget);
            //robot.rightDriveB.setTargetPosition(newRightTarget);

            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDriveF.setPower(speed);
            robot.rightDriveF.setPower(speed);
            robot.leftDriveB.setPower(speed);
            robot.rightDriveB.setPower(speed);


            positionLeftDriveF = robot.leftDriveF.getCurrentPosition();
            positionLeftDriveB = robot.leftDriveB.getCurrentPosition();
            rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
            rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   // (robot.leftDriveF.isBusy() && robot.rightDriveF.isBusy() && robot.leftDriveB.isBusy() && robot.rightDriveB.isBusy())) {
                    //(robot.leftDriveF.isBusy() && robot.leftDriveB.isBusy())) {
                    (rangLeftDriveF > 150) && (rangLeftDriveB > 150)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDriveF.setPower(leftSpeed);
                robot.rightDriveF.setPower(rightSpeed);
                robot.leftDriveB.setPower(leftSpeed);
                robot.rightDriveB.setPower(rightSpeed);

                // Display drive status for the driver.
                /*telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDriveF.getCurrentPosition(),
                        robot.rightDriveF.getCurrentPosition());
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDriveB.getCurrentPosition(),
                        robot.rightDriveB.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();*/

                positionLeftDriveF = robot.leftDriveF.getCurrentPosition();
                positionLeftDriveB = robot.leftDriveB.getCurrentPosition();
                rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
                rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);

                Log.d(TAG, "Err/St" + " " + error + " " + steer);
                Log.d(TAG, "Target" + " " + newLeftFTarget + " " + newLeftBTarget);
                Log.d(TAG, "Actual" + " " + robot.leftDriveF.getCurrentPosition() + " " + robot.rightDriveF.getCurrentPosition());
                Log.d(TAG, "Actual" + " " + robot.leftDriveB.getCurrentPosition() + " " + robot.rightDriveB.getCurrentPosition());
                Log.d(TAG, "Speed" + " " + leftSpeed + " " + rightSpeed);
            }

            // Stop all motion;
            robot.leftDriveF.setPower(0);
            robot.rightDriveF.setPower(0);
            robot.leftDriveB.setPower(0);
            robot.rightDriveB.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

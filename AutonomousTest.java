package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.PushbotAutoDriveByGyro_Linear_Connection.HEADING_THRESHOLD;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="AutonomousTest", group="Pushbot")
@Disabled
public class AutonomousTest extends LinearOpMode {

    ImageProcessingBlue imageProcessing = new ImageProcessingBlue();
    PushbotAutoDriveByEncoder_Linear_Connection pushbotAutoDriveByEncoderLinearConnection = new PushbotAutoDriveByEncoder_Linear_Connection();
    ConnectionHardware robot       = new ConnectionHardware();
    private ElapsedTime     runtime = new ElapsedTime();
    ImageProcessingBlue.DuckPositionBlue duckPosition;
    private String TAG = "AutonomousConnection";
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1; //0.15;     // Larger is more responsive, but also less stable
    static final double     MINIMUM_SPEED           = 0.1;
    static final double     HEADING_THRESHOLD       = 1 ;

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

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        waitForStart();
        robot.turnArmMotor.setPower(0);
        robot.turnArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData(">", "Robot start");
        telemetry.update();
        //duckPosition = ImageProcessingRed.DuckPositionRed.CENTER;
        duckPosition = imageProcessing.FindDuckPosition();
        duckPosition = ImageProcessingBlue.DuckPositionBlue.CENTER;
        telemetry.addData(">", "Robot start 1");
        telemetry.update();
        /*Log.d(TAG, "DuckPosition = " + duckPosition);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT) {
            telemetry.addLine("The Duck Position is Left");
            Log.d(TAG, "The Duck Position is Left");
            driveToTheShippingHub(ImageProcessingBlue.DuckPositionBlue.LEFT);
            //sleep(1000);
            //droppingCubeOnTheShippinghubWithTheArm(ImageProcessingRed.DuckPositionRed.LEFT);
            //putTheCubeOnTheShippingHub(ImageProcessingRed.DuckPositionRed.LEFT);
            driveToTheCarrousel();
            driveToTheWarehouses();
            //settlingInAWarehouse();

        }

        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER) {
            telemetry.addLine("The Duck Position is Center");
            Log.d(TAG, "The Duck Position is Center");
            driveToTheShippingHub(ImageProcessingBlue.DuckPositionBlue.CENTER);
            //sleep(1000);
            //droppingCubeOnTheShippinghubWithTheArm(ImageProcessingRed.DuckPositionRed.LEFT);
            //putTheCubeOnTheShippingHub(ImageProcessingRed.DuckPositionRed.CENTER);
            driveToTheCarrousel();
            driveToTheWarehouses();
            //settlingInAWarehouse();
        }
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.RIGHT) {
            telemetry.addLine("The Duck Position is Right");
            Log.d(TAG, "The Duck Position is Right");
            driveToTheShippingHub(ImageProcessingBlue.DuckPositionBlue.RIGHT);
            //droppingCubeOnTheShippinghubWithTheArm(ImageProcessingRed.DuckPositionRed.RIGHT);
            //putTheCubeOnTheShippingHub(ImageProcessingRed.DuckPositionRed.RIGHT);
            driveToTheCarrousel();
            driveToTheWarehouses();
            //settlingInAWarehouse();
        }*/
        telemetry.update();
        //while(opModeIsActive());

        int armTargetPosition  = 1000;
        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(-0.8);
        runtime.reset();
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition && runtime.seconds() < 8)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(0);

            Log.d(TAG, "rightDriveF Motor position is =" + robot.rightDriveF.getCurrentPosition());
            Log.d(TAG, "leftDriveF Motor position is =" + robot.leftDriveF.getCurrentPosition());
            Log.d(TAG, "rightDriveB Motor position is =" + robot.rightDriveB.getCurrentPosition());
            Log.d(TAG, "leftDriveB Motor position is =" + robot.leftDriveB.getCurrentPosition());
        }

    private void driveToTheShippingHub (ImageProcessingBlue.DuckPositionBlue duckPosition)
    {
        robot.collectorMotor.setPower(-0.2);
        gyroDrive(1.0, 72, 0);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.RIGHT)
        {
            gyroDrive(1.0, 9, 0);
        }
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT) {
            gyroDrive(1.0, 22, 0);
        }
        droppingCubeOnTheShippinghubWithTheArm(duckPosition);
        gyroTurn(0.5, 0);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            robot.sideDrive(-0.5);
            sleep(400);
            robot.sideDrive(0);
        }
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            robot.sideDrive(-0.5);
            sleep(400);
            robot.sideDrive(0);
        }
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.RIGHT)
        {
            //robot.sideDrive(-0.5);
            //sleep(1000);
            //robot.sideDrive(0);
            gyroTurn(0.5, 0);
            gyroDrive(1.0, -45, 0);

        }
        else {
            if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
            {
                gyroDrive(1.0, -40, 0);
            }

            else {
                gyroDrive(1.0, -37, 0);
            }
        }
    }
    private void driveToTheCarrousel ()
    {
        gyroTurn(0.5, 0);
        gyroDrive(0.5, -57, 0);
        gyroTurn(0.5, -90);
        gyroDrive(0.5, -53, -90);
        robot.carrouselMotor.setPower(0.75);
        sleep(2900);
        robot.carrouselMotor.setPower(0.0);

    }

    private void driveToTheWarehouses () {
        gyroTurn(0.5, -53);
        gyroDrive(1.0, 35,-53);
        gyroTurn(0.8, -90);
        gyroDrive(1.0, 120, -90);
        driveOnTime();
    }

    private void driveOnTime ()
    {
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveF.setPower(0.8);
            robot.leftDriveB.setPower(0.8);
            robot.rightDriveF.setPower(0.8);
            robot.rightDriveB.setPower(0.8);
            sleep(1400);
            robot.leftDriveF.setPower(0.0);
            robot.leftDriveB.setPower(0.0);
            robot.rightDriveF.setPower(0.0);
            robot.rightDriveB.setPower(0.0);
        }
        else {
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveF.setPower(0.8);
            robot.leftDriveB.setPower(0.8);
            robot.rightDriveF.setPower(0.8);
            robot.rightDriveB.setPower(0.8);
            sleep(1150);
            robot.leftDriveF.setPower(0.0);
            robot.leftDriveB.setPower(0.0);
            robot.rightDriveF.setPower(0.0);
            robot.rightDriveB.setPower(0.0);
        }
    }
    private void droppingCubeOnTheShippinghubWithTheArm (ImageProcessingBlue.DuckPositionBlue duckPosition)
    {
        int armTargetPosition  = 1000;
        gyroTurn(0.5, 180);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {

            robot.sideDrive(0.6);
            sleep(800);
            robot.sideDrive(0);

        }
        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(0.7);
        runtime.reset();
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition && runtime.seconds() < 8)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(0);
        sleep(1000);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            armTargetPosition = 1753;
        }
        else if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            armTargetPosition = 1535;
            robot.sideDrive(0.5);
            sleep(510);
            robot.sideDrive(0);
        }
        else
        {
            armTargetPosition = 1240;
            robot.sideDrive(-0.5);
            sleep(600);
            robot.sideDrive(0);
            gyroTurn(0.5, 180);
        }
        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(0.4);
        runtime.reset();
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition && runtime.seconds() < 8)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(0);

        Log.d(TAG, "armMotor position is (stop) " + " " + robot.armMotor.getCurrentPosition());
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.RIGHT)
        {
            robot.droppingACube.setPosition(-0.6);
            robot.collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.collectorMotor.setPower(0.3);
        }
        else if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {

            robot.sideDrive(-0.5);
            sleep(715);
            robot.sideDrive(0);
            robot.collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.collectorMotor.setPower(0.35);
        }
        else {
            //robot.sideDrive(-0.5);
            //sleep(1262);
            //robot.sideDrive(0);
            gyroTurn(0.5, 180);
            robot.collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.collectorMotor.setPower(1.0);
        }
        Log.d(TAG, "collectorMotor position is " + " " + robot.collectorMotor.getCurrentPosition());
        runtime.reset();
        while (robot.collectorMotor.getCurrentPosition() < 900 && runtime.seconds() < 2)
        {
            robot.collectorMotor.getCurrentPosition();
            Log.d(TAG, "collectorMotor position is (wait) " + " " + robot.collectorMotor.getCurrentPosition());
        }
        robot.collectorMotor.setPower(0.0);
        robot.droppingACube.setPosition(0.75);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            robot.sideDrive(0.5);
            sleep(1350);
            robot.sideDrive(0);
        }
        sleep(1000);
        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(-0.5);
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() > 0)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(0.0);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.RIGHT)
        {
            robot.sideDrive(0.5);
            sleep(750);
            robot.sideDrive(0);
        }
    }
    /*private void putTheCubeOnTheShippingHub (ImageProcessingBlue.DuckPositionBlue duckPositionBlue)
    {
        int armTargetPosition;
        if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            armTargetPosition = 1885
            ;
        }
        else if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            armTargetPosition = 1645;
        }
        else
        {
            armTargetPosition = 1460;
        }
 
        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(0.5);
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(-0.1);
        Log.d(TAG, "armMotor position is (stop) " + " " + robot.armMotor.getCurrentPosition());
        if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.LEFT) {
            gyroDrive(1.0, 17, 0);
        }
        else if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.CENTER) {
            gyroDrive(1.0, 7, 0);
        }
        //robot.collectorMotor.setPower(0.37);
        //sleep(3500);
        //robot.collectorMotor.setPower(0);
        releaseTheCube();
        if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.CENTER) {
            robot.armMotor.setPower(-0.5);
            sleep(3200);
            robot.armMotor.setPower(0);
        }
        else if (duckPositionBlue == ImageProcessingBlue.DuckPositionBlue.LEFT) {
            robot.armMotor.setPower(-0.5);
            sleep(2800);
            robot.armMotor.setPower(0);
        }
        else {
            robot.armMotor.setPower(-0.5);
            sleep(2000);
            robot.armMotor.setPower(0);
        }
    }*/
    private void settlingInAWarehouse ()
    {
        gyroTurn(0.5, 180);
        int armTargetPosition;
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            armTargetPosition = 1150;
        }
        else if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            armTargetPosition = 1150;
        }
        else
        {
            armTargetPosition = 1150;
        }

        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(0.6);
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(-0.1);
        Log.d(TAG, "armMotor position is (stop) " + " " + robot.armMotor.getCurrentPosition());
        robot.turnArmMotor.setTargetPosition(55);
        robot.turnArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turnArmMotor.setPower(0.25);
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT)
        {
            armTargetPosition = 1720;
        }
        else if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER)
        {
            armTargetPosition = 1720;
        }
        else
        {
            armTargetPosition = 1720;
        }

        robot.armMotor.getCurrentPosition();
        Log.d(TAG, "armMotor position is (start)" + " " + robot.armMotor.getCurrentPosition());
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setPower(0.6);
        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < armTargetPosition)
        {
            Log.d(TAG, "armMotor position is (wait) " + " " + robot.armMotor.getCurrentPosition());
        }
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(-0.1);
        Log.d(TAG, "armMotor position is (stop) " + " " + robot.armMotor.getCurrentPosition());
    }

    private void releaseTheCube () {
        robot.collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.collectorMotor.setTargetPosition(770);
        robot.collectorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (duckPosition == ImageProcessingBlue.DuckPositionBlue.LEFT) {
            robot.collectorMotor.setPower(0.7);
        }
        else if (duckPosition == ImageProcessingBlue.DuckPositionBlue.CENTER) {
            robot.collectorMotor.setPower(0.3);
        }
        else
        {
            robot.collectorMotor.setPower(0.3);
        }
        while (opModeIsActive() &&
                (runtime.seconds() < 2.5) &&
                (robot.collectorMotor.isBusy())) {
            Log.d(TAG, "collectorMotor is " + " " + robot.collectorMotor.getCurrentPosition());
        }
        if (robot.collectorMotor.isBusy())
        {
            Log.d(TAG, "timeout 3 seconds");
        }
        else
        {
            Log.d(TAG, "collectorMotor in the target position");
        }
        robot.collectorMotor.setPower(0);
    }
    public void gyroTurn (  double speed, double angle) {
        angle = -angle;
        runtime.reset();
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

        if ((Math.abs(error) <= HEADING_THRESHOLD) || (runtime.seconds() > 3)) {
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
        int     newRightFTarget;
        int     newRightBTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int positionLeftDriveB;
        int positionRightDriveB;
        int rangLeftDriveB;
        int rangRightDriveB;
        int direction = 1;

        angle = -angle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

             distance = robot.cm_to_inch(distance);
             if (distance < 0)
             {
                 direction = -1;
             }
             Log.d(TAG, "the distance is (inch) = " + distance);
             Log.d(TAG, "the angle is =" + " " + angle);
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftBTarget = robot.leftDriveB.getCurrentPosition() + moveCounts;
            //newRightFTarget = robot.rightDriveF.getCurrentPosition() + moveCounts;
            //newLeftBTarget = robot.leftDriveB.getCurrentPosition() + moveCounts;
            newRightBTarget = robot.rightDriveB.getCurrentPosition() + moveCounts;
            Log.d(TAG, "move counts = " + " " + moveCounts);
            Log.d(TAG, "new left BACK target =" + " " + newLeftBTarget);
            Log.d(TAG, "new right BACK target =" + " " + newRightBTarget);
            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftDriveF.setTargetPosition(newLeftFTarget);
            //robot.rightDriveF.setTargetPosition(newRightTarget);
            robot.leftDriveB.setTargetPosition(newLeftBTarget);
            robot.rightDriveB.setTargetPosition(newRightBTarget);

            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            speed = speed * direction;
            robot.leftDriveF.setPower(speed);
            robot.rightDriveF.setPower(speed);
            robot.leftDriveB.setPower(speed);
            robot.rightDriveB.setPower(speed);


            positionLeftDriveB = robot.leftDriveB.getCurrentPosition();
            positionRightDriveB = robot.rightDriveB.getCurrentPosition();
            rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
            rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   // (robot.leftDriveF.isBusy() && robot.rightDriveF.isBusy() && robot.leftDriveB.isBusy() && robot.rightDriveB.isBusy())) {
                    //(robot.leftDriveF.isBusy() && robot.leftDriveB.isBusy())) {
                    (rangLeftDriveB > 150) && (rangRightDriveB > 150)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                //if (distance < 0)
                 //    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                if (Math.abs(leftSpeed) < MINIMUM_SPEED)
                {
                    if (leftSpeed >= 0)
                    {
                        leftSpeed = MINIMUM_SPEED;
                    }
                    else
                    {
                        leftSpeed = -MINIMUM_SPEED;
                    }


                }

                if (Math.abs(rightSpeed) < MINIMUM_SPEED)
                {
                    if (rightSpeed >= 0)
                    {
                        rightSpeed = MINIMUM_SPEED;
                    }
                    else {
                        rightSpeed = -MINIMUM_SPEED;
                    }

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

                positionLeftDriveB = robot.leftDriveB.getCurrentPosition();
                positionRightDriveB = robot.rightDriveB.getCurrentPosition();
                rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
                rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);

                Log.d(TAG, "Err/St" + " " + error + " " + steer);
                Log.d(TAG, "Target" + " " + newLeftBTarget + " " + newRightBTarget);
                Log.d(TAG, "Actual" + " " + robot.rightDriveB.getCurrentPosition() + " " + robot.leftDriveB.getCurrentPosition());
                Log.d(TAG, "Actual" + " " + robot.leftDriveF.getCurrentPosition() + " " + robot.rightDriveF.getCurrentPosition());
                Log.d(TAG, "Rang" + " " + rangRightDriveB + " " + rangLeftDriveB);
                Log.d(TAG, "Speed" + " " + leftSpeed + " " + rightSpeed);
            }

            // Stop all motion;
            robot.leftDriveF.setPower(0);
            robot.rightDriveF.setPower(0);
            robot.leftDriveB.setPower(0);
            robot.rightDriveB.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

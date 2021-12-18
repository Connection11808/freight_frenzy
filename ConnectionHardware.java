/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class ConnectionHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDriveF   = null;
    public DcMotor  rightDriveF  = null;
    public DcMotor  leftDriveB   = null;
    public DcMotor  rightDriveB  = null;
    public DcMotor  elevatorsMotor = null;
    public DcMotor  carrouselMotor = null;
    public Servo servoDoorF = null;
    public Servo servoDoorB = null;
    // The IMU sensor object
    private BNO055IMU imu;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double ARM_HOME = 0.0;
    public static final double ARM_MIN_RANGE = 0.0;
    public static final double ARM_MAX_RANGE = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ConnectionHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveF  = hwMap.get(DcMotor.class, "LDF");
        rightDriveF = hwMap.get(DcMotor.class, "RDF");
        leftDriveB  = hwMap.get(DcMotor.class, "LDB");
        rightDriveB = hwMap.get(DcMotor.class, "RDB");
        //elevatorsMotor = hwMap.get(DcMotor.class, "EM");
        carrouselMotor = hwMap.get(DcMotor.class, "CM");
        //servoDoorF = hwMap.get(Servo.class, "SDF");
        //servoDoorB = hwMap.get(Servo.class, "SDB");

        leftDriveF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftDriveB.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        //elevatorsMotor.setPower(0);
        carrouselMotor.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elevatorsMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carrouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //servoDoorF.setPosition(ARM_HOME);
        //servoDoorB.setPosition(ARM_HOME);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        imu = hwMap.get(BNO055IMU.class, "imu ");
        initImu();


        }
    public void sideDrive(double speed){
        rightDriveF.setPower(-speed);
        rightDriveB.setPower(speed);
        leftDriveF.setPower(speed);
        leftDriveB.setPower(-speed);
    }

    private boolean initImu(){
        boolean sucses;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        sucses = imu.initialize(parameters);
        return sucses;


    }

    public float GetImuAngle(){

        Orientation angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    public double cm_to_inch (double cm)
    {
        double inch = cm * 0.393701;
        return(inch);

    }

}


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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.DalvCode;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop_Tank_Connection", group="Pushbot")

public class PushbotTeleopTank_Iterative_Connection extends OpMode{

    /* Declare OpMode members. */
    ConnectionHardware robot       = new ConnectionHardware(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double      armPosition  = robot.ARM_HOME;
    final double ARM_SPEED = 0.01;
    boolean armMotorStart = false;
    public String TAG = "teleopTank";
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double left_y = -gamepad1.left_stick_y;
        double right_y = -gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;

        /*if (gamepad2.a) {
            robot.plierServo.setPosition(1);
        }
        else if (gamepad2.b) {
            robot.plierServo.setPosition(-1);
        }*/

        if (gamepad2.x) {
            robot.droppingACube.setPosition(0.5);
        } else if (gamepad2.y) {
            robot.droppingACube.setPosition(-1);
        }


        if (gamepad2.right_trigger == 1.0) {
            robot.collectorMotor.setPower(0.6);
        } else if (gamepad2.left_trigger == 1.0) {
            robot.collectorMotor.setPower(-1.0);
        } else {
            robot.collectorMotor.setPower(0.0);
            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad2.left_bumper) {
            //robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setPower(0.55);
            armMotorStart = true;
            robot.armMotor.getCurrentPosition();
            Log.d(TAG, "arm position is (left) " + " " + robot.armMotor.getCurrentPosition());
        }
        else if (gamepad2.right_bumper) {
            //robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setPower(-0.7);
            armMotorStart = true;
            robot.armMotor.getCurrentPosition();
            Log.d(TAG, "arm position is (right) " + " " + robot.armMotor.getCurrentPosition());
        }
        else {

            /*]

             */

                /*if (armMotorStart == true) {
                    if (armMotor_run_to_position == false) {
                        armPosition = robot.armMotor.getCurrentPosition();
                        if (armPosition < 0) {
                            armPosition -= 100;
                        } else {
                            armPosition += 100;
                        }
                        Log.d(TAG, "Set arm Position to = " + " " + armPosition);
                        robot.armMotor.setTargetPosition(armPosition);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor_run_to_position = true;
                    } else {
                    /*
                }
                    if (robot.armMotor.isBusy() == false)
                    {
                    }
                        Log.d(TAG, "armMotor not Busy"); */
            // armMotor_run_to_position = false;
            //robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //armMotorStart = false;
            //robot.armMotor.setPower(0);
            //}
            //else {
            //sleep(100);
            //robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Log.d(TAG, "armMotor is Busy");
            //Log.d(TAG, "arm position is " + " " + robot.armMotor.getCurrentPosition());
            //  armMotorStart = false;
            //}

            robot.armMotor.setPower(-0.1);
            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.x) {
            robot.carrouselMotor.setPower(-0.8);
        } else if (gamepad1.y) {
            robot.carrouselMotor.setPower(0.8);
        } else {
            robot.carrouselMotor.setPower(0);
        }

        if (gamepad1.right_bumper) {
            robot.sideDrive(0.8);
        } else if (gamepad1.left_bumper) {
            robot.sideDrive(-0.8);
        } else {
            robot.rightDriveF.setPower(0);
            robot.leftDriveF.setPower(0);
            robot.rightDriveB.setPower(0);
            robot.leftDriveB.setPower(0);

        }

        robot.leftDriveF.setPower(left_y);
        robot.rightDriveF.setPower(right_y);
        robot.leftDriveB.setPower(left_y);
        robot.rightDriveB.setPower(right_y);

    }
    // Use gamepad left & right Bumpers to open and close the claw
        /*if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            0.clawOffset -= CLAW_SPEED;*/

    // Move both servos to new position.  Assume servos are mirror image of each other.
    //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        /*robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        else
            robot.leftArm.setPower(0.0);*/

    // Send telemetry message to signify robot running;



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
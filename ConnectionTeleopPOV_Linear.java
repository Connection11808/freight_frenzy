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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop POV_Connection", group="Pushbot")

public class ConnectionTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    ConnectionHardware robot = new ConnectionHardware();   // Use a Pushbot's hardware
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    public String TAG = "teleop";

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean armMotorStart = false;
        boolean collectionMotorStart = false;
        int armPosition;
        boolean armMotor_run_to_position = false;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "Hello Driver");    //
        Log.d("teleop", "Say Hello Driver");
        //telemetry.update();
        Log.d("teleop", "update");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                right /= max;
                left /= max;
            }

            robot.leftDriveF.setPower(left);
            robot.leftDriveB.setPower(left);
            robot.rightDriveF.setPower(right);
            robot.rightDriveB.setPower(right);
            Log.d(TAG, "Left" + " " + left);
            Log.d(TAG, "Right" + " " + right);
            Log.d(TAG, "turn" + " " + turn);
            // Output the safe vales to the motor drives.
            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;*/

            if (gamepad2.a) {
                robot.plierServo.setPosition(1);
            } else if (gamepad2.b) {
                robot.plierServo.setPosition(-1);
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
                robot.armMotor.setPower(0.4);
                armMotorStart = true;
                robot.armMotor.getCurrentPosition();
                Log.d(TAG, "arm position is (left) " + " " + robot.armMotor.getCurrentPosition());
            }
            else if (gamepad2.right_bumper) {
                //robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-0.4);
                armMotorStart = true;
                robot.armMotor.getCurrentPosition();
                Log.d(TAG, "arm position is (right) " + " " + robot.armMotor.getCurrentPosition());
            } else {
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

                    if (gamepad2.x) {
                        robot.carrouselMotor.setPower(-0.8);
                    } else if (gamepad2.y) {
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

                    // Move both servos to new position.  Assume servos are mirror image of each other.
                    //clawOffset = Range.clip(clawOffset, -0.5, 0.5);

                    // Use gamepad buttons to move arm up (Y) and down (A)


                    // Send telemetry message to signify robot running;
            /*telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();*/


                    //Log.d("teleop", "claw" + "Offset = %.2f" + clawOffset);
                    //Log.d("teleop", "left" + "%.2f" + left);
                    //Log.d("teleop", "right" + "%.2f" + right);
                    // Pace this loop so jaw action is reasonable
                }
            }
        }


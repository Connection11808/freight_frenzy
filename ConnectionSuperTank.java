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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="ConnectionSuperTank", group="Pushbot")

public class ConnectionSuperTank extends LinearOpMode {

    /* Declare OpMode members. */
    ConnectionHardware robot = new ConnectionHardware();   // Use a Pushbot's hardware
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    public String TAG = "teleop";

    @Override
    public void runOpMode() {
        double left;
        double right;
        double side;
        double max;
        boolean armMotorStart = false;
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
            right = -gamepad1.right_stick_y;
            left  = -gamepad1.left_stick_y;
            side = gamepad1.right_stick_x;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            robot.leftDriveF.setPower(left -side);
            robot.leftDriveB.setPower(left + side);
            robot.rightDriveF.setPower(right + side);
            robot.rightDriveB.setPower(right - side);


            // Output the safe vales to the motor drives.
            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;*/

            /*if (gamepad2.a) {
                robot.plierServo.setPosition(1);
            } else if (gamepad2.b) {
                robot.plierServo.setPosition(-1);
            }*/

            if (gamepad2.right_trigger == 1.0) {
                robot.collectorMotor.setPower(0.25);
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
        }
    }
}


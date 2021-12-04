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

package org.firstinspires.ftc.freight_frenzy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * {@link UnitTestConnection} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@TeleOp(name = "Unit Test", group = "Concept")
public class UnitTestConnection extends LinearOpMode  {

    private String TAG = "UnitTest";
    private ConnectionHardware robot = new ConnectionHardware();
    private double MotorSpeed = 0.8;

    @Override public void runOpMode() {
        Log.d(TAG, "Wait for Start");
        robot.init(hardwareMap);
        while (!isStarted()) {

        }
        Log.d(TAG, "Start play");

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.y == true) {
                robot.rightDriveF.setPower(MotorSpeed);
                Log.d(TAG, "rightDriveF Motor position is =" + robot.rightDriveF.getCurrentPosition());
            } else {
                robot.rightDriveF.setPower(0);
            }

            if (gamepad1.a == true) {
                robot.rightDriveB.setPower(MotorSpeed);
                Log.d(TAG, "rightDriveB Motor position is =" + robot.rightDriveB.getCurrentPosition());
            } else {
                robot.rightDriveB.setPower(0);
            }

            if (gamepad1.x == true) {
                robot.leftDriveF.setPower(MotorSpeed);
                Log.d(TAG, "leftDriveF Motor position is =" + robot.leftDriveF.getCurrentPosition());
            } else {
                robot.leftDriveF.setPower(0);
            }

            if (gamepad1.b == true) {
                robot.leftDriveB.setPower(MotorSpeed);
                Log.d(TAG, "leftDriveB Motor position is =" + robot.leftDriveB.getCurrentPosition());
            }
            else
                {
                    robot.leftDriveB.setPower(0);
                }

                if (gamepad1.right_bumper == true) {
                    MotorSpeed = -MotorSpeed;
                    Log.d(TAG, "The speed is = " + MotorSpeed);
                }

                if (gamepad1.left_bumper == true) {
                    robot.leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Log.d(TAG, "Reset Encoder all Motors");
                }
            }
        }


}
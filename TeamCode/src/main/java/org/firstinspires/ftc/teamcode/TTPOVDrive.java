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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TT POV Drive", group = "TT")
public class TTPOVDrive extends TTLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new TTHardware();
        robot.init(hardwareMap);

        robot.motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLanding.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLanding.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        // make sure the gyro is calibrated.
        while (robot.gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }
        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        double leftStickY = 0;
        double leftStickX = 0;
        double rightStickX = 0;

        double robotHeadingRad = 0.0;
        double powerCompY = 0.0;
        double powerCompX = 0.0;

        double powerFrontLeft = 0.0;
        double powerFrontRight = 0.0;
        double powerRearLeft = 0.0;
        double powerRearRight = 0.0;
        boolean isArmLowered = false;
        boolean isRightExtended = false;
        boolean isLifted = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Gamepad 1 - Driving
            if (gamepad1.left_stick_y != 0) {
                leftStickY = stepInput(-gamepad1.left_stick_y);
            } else if (gamepad1.dpad_up) {
                leftStickY = 0.25;
            } else if (gamepad1.dpad_down) {
                leftStickY = -0.25;
            } else {
                leftStickY = 0.0;
            }
            if (gamepad1.left_stick_x != 0) {
                leftStickX = stepInput(gamepad1.left_stick_x);
            } else if (gamepad1.dpad_right) {
                leftStickX = 0.25;
            } else if (gamepad1.dpad_left) {
                leftStickX = -0.25;
            } else {
                leftStickX = 0.0;
            }
            rightStickX = stepInputRotate(gamepad1.right_stick_x);

            if (leftStickY != 0 || leftStickX != 0 || rightStickX != 0) {
                robotHeadingRad = Math.toRadians(((360 - robot.gyro.getHeading()) % 360));
                powerCompY = (Math.cos(robotHeadingRad) * leftStickY) + (Math.sin(robotHeadingRad) * leftStickX);
                powerCompX = -(Math.sin(robotHeadingRad) * leftStickY) + (Math.cos(robotHeadingRad) * leftStickX);

                powerFrontLeft = powerCompY + powerCompX + rightStickX;
                powerFrontRight = -powerCompY + powerCompX + rightStickX;
                powerRearLeft = powerCompY - powerCompX + rightStickX;
                powerRearRight = -powerCompY - powerCompX + rightStickX;
            } else {
                powerFrontLeft = 0.0;
                powerFrontRight = 0.0;
                powerRearLeft = 0.0;
                powerRearRight = 0.0;
            }

            robot.motorFrontLeft.setPower(Range.clip(powerFrontLeft, -1.0, 1.0));
            robot.motorFrontRight.setPower(Range.clip(powerFrontRight, -1.0, 1.0));
            robot.motorRearLeft.setPower(Range.clip(powerRearLeft, -1.0, 1.0));
            robot.motorRearRight.setPower(Range.clip(powerRearRight, -1.0, 1.0));

            // Gamepad 2 - Dpad Up and Dpad Down
            if (gamepad2.dpad_up) {
                if (robot.motorExtend.getCurrentPosition() < -5000 && robot.limitExtend.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorExtend.setPower(0.0);
                } else {
                    robot.motorExtend.setPower(0.5);
                }
            } else if (gamepad2.dpad_down) {
                if (robot.motorExtend.getCurrentPosition() > 0 && robot.limitExtend.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorExtend.setPower(0.0);
                } else {
                    robot.motorExtend.setPower(-0.5);
                }
            } else {
                robot.motorExtend.setPower(0.0);
            }

            if (gamepad2.left_trigger > 0.1) {   // raising beam, lower robot
                if (robot.limitLanding.getState() == LIMIT_MAG_ON && robot.motorLanding.getCurrentPosition() > -400 && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else {
                    robot.motorLanding.setPower(-0.5);
                }
            } else if (gamepad2.right_trigger > 0.1) {   // lowering beam, lift robot
                if (robot.limitLanding.getState() == LIMIT_MAG_ON && robot.motorLanding.getCurrentPosition() < -400 && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else {
                    robot.motorLanding.setPower(0.5);
                }
            } else {
                robot.motorLanding.setPower(0.0);
            }
/*
            // Gamepad 2 - Left Trigger and Right Trigger
            if (gamepad2.left_trigger > 0.1) {   // lower robot
                if (robot.motorLanding.getCurrentPosition() < -1000 && robot.limitLanding.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else if (robot.motorLanding.getCurrentPosition() < -13000) {
                    robot.motorLanding.setPower(0.3);
                } else if (robot.motorLanding.getCurrentPosition() < -11000) {
                    robot.motorLanding.setPower(0.5);
                } else {
                    robot.motorLanding.setPower(1.0);
                }
            } else if (gamepad2.right_trigger > 0.1) {   // lift robot
                if (robot.motorLanding.getCurrentPosition() > -13000 && robot.limitLanding.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else if (robot.motorLanding.getCurrentPosition() > -1000) {
                    robot.motorLanding.setPower(-0.3);
                } else if (robot.motorLanding.getCurrentPosition() > -3000) {
                    robot.motorLanding.setPower(-0.5);
                } else {
                    robot.motorLanding.setPower(-1.0);
                }
            } else {
                robot.motorLanding.setPower(0.0);
            }
*/
            // Gamepad 2 - Y and A
            if (gamepad2.y) {
                if (robot.limitLiftTop.getState() == LIMIT_MEC_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLift.setPower(0.0);
                } else {
                    robot.motorLift.setPower(-1.0);
                }
            } else if (gamepad2.a) {
                if (robot.limitLiftBottom.getState() == LIMIT_MEC_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLift.setPower(0.0);
                } else {
                    robot.motorLift.setPower(1.0);
                }
            } else {
                robot.motorLift.setPower(0.0);
            }

            // Gamepad 2 - X
            if (gamepad2.x) {
                robot.servoScreen.setPosition(0.8);
            } else {
                robot.servoScreen.setPosition(0.2);
            }

            // Gamepad 2 - B
            if (gamepad2.b) {
                robot.servoMarker.setPosition(0.9);
            } else {
                robot.servoMarker.setPosition(0.3);
            }

            // Gamepad 1 - Left Bumper and Right Bumper for lifter
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                // Reset Gyro
                telemetry.addData(">", "Gyro Calibrating. Do Not move!");
                telemetry.update();
                robot.gyro.calibrate();
                // make sure the gyro is calibrated.
                while (robot.gyro.isCalibrating())  {
                    Thread.sleep(50);
                    idle();
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading()
                    + " | 360 - Heading: " + ((360 - robot.gyro.getHeading()) % 360));
            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)"
                    , powerFrontLeft, powerFrontRight, powerRearLeft, powerRearRight);
            telemetry.addData("Landing", "Pwr: (%.2f) | Pos: " + robot.motorLanding.getCurrentPosition()
                    + " | Limit: " + (robot.limitLanding.getState() == LIMIT_MAG_ON ? "ON" : "OFF"), robot.motorLanding.getPower());
            telemetry.addData("Lift", "Pos: " + robot.motorLift.getCurrentPosition()
                    + " | Limit Btm: " + (robot.limitLiftBottom.getState() == LIMIT_MEC_ON ? "ON" : "OFF")
                    + " | Limit Top: " + (robot.limitLiftTop.getState() == LIMIT_MEC_ON ? "ON" : "OFF"));
            telemetry.addData("Extend", "Pos: " + robot.motorExtend.getCurrentPosition()
                    + " | Limit: " + (robot.limitExtend.getState() == LIMIT_MAG_ON ? "ON" : "OFF")
                    + " | Screen %.2f", robot.servoScreen.getPosition());
            telemetry.addData("Sensor", "ClrBtm: " + isGold(robot.sensorColorBottom)
                    + " | DstBtm: %.2f | ClrFnt: " + isGold(robot.sensorColorFront)
                    + " | DstFnt: %.2f | Marker: %.2f", robot.sensorDistBottom.getDistance(DistanceUnit.CM)
                    , robot.sensorDistFront.getDistance(DistanceUnit.CM), robot.servoMarker.getPosition());
            telemetry.addData("Range", "Left: " + robot.rangeLeft.getDistance(DistanceUnit.CM)
                    + " | Rear: " + robot.rangeRear.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}


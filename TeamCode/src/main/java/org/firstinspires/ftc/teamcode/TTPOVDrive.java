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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@TeleOp(name = "TT POV Drive", group = "TT")
public class TTPOVDrive extends TTLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    // State used for updating telemetry
    private Orientation angles1;
    private Orientation angles2;

    private double leftStickY = 0;
    private double leftStickX = 0;
    private double rightStickX = 0;

    private double robotHeadingRad = 0.0;
    private double powerCompY = 0.0;
    private double powerCompX = 0.0;

    private double powerFrontLeft = 0.0;
    private double powerFrontRight = 0.0;
    private double powerRearLeft = 0.0;
    private double powerRearRight = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new TTHardware();
        robot.init(hardwareMap);

        robot.motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLanding.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


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
//                robotHeadingRad = Math.toRadians(((360 - robot.gyro.getHeading()) % 360));
                robotHeadingRad = Math.toRadians(getRobotHeading());
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
                if (robot.motorExtend.getCurrentPosition() < -4500 && robot.limitExtend.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorExtend.setPower(0.0);
                } else if (robot.motorExtend.getCurrentPosition() < -3500) {
                    robot.motorExtend.setPower(0.2);
                } else {
                    robot.motorExtend.setPower(0.5);
                }
            } else if (gamepad2.dpad_down) {
                if (robot.motorExtend.getCurrentPosition() > -500 && robot.limitExtend.getState() == LIMIT_MAG_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorExtend.setPower(0.0);
                } else if (robot.motorExtend.getCurrentPosition() > -1500) {
                    robot.motorExtend.setPower(-0.2);
                } else {
                    robot.motorExtend.setPower(-0.5);
                }
            } else {
                robot.motorExtend.setPower(0.0);
            }

            // Gamepad 2 - Left Trigger and Right Trigger
            if (gamepad2.left_trigger > 0.1) {   // raising beam, lower robot
                if (robot.limitLanding.getState() == LIMIT_MAG_ON && robot.motorLanding.getCurrentPosition() > 700 && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else if (robot.motorLanding.getCurrentPosition() > 550) {
                    robot.motorLanding.setPower(-0.3);
                } else {
                    robot.motorLanding.setPower(-0.5);
                }
            } else if (gamepad2.right_trigger > 0.1) {   // lowering beam, lift robot
                if (robot.limitLanding.getState() == LIMIT_MAG_ON && robot.motorLanding.getCurrentPosition() < 100 && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLanding.setPower(0.0);
                } else if (robot.motorLanding.getCurrentPosition() > 250) {
                    robot.motorLanding.setPower(0.3);
                } else {
                    robot.motorLanding.setPower(0.5);
                }
            } else {
                robot.motorLanding.setPower(0.0);
            }

            // Gamepad 2 - Y and A
            if (gamepad2.y) {   // lift bucket
                if (robot.limitLiftTop.getState() == LIMIT_MEC_ON && (!gamepad2.left_bumper || !gamepad2.right_bumper)) {
                    robot.motorLift.setPower(0.0);
                } else {
                    robot.motorLift.setPower(-1.0);
                }
            } else if (gamepad2.a) {   // lower bucket
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
                /*
                // Reset Gyro
                telemetry.addData(">", "Gyro Calibrating. Do Not move!");
                telemetry.update();
                robot.gyro.calibrate();
                // make sure the gyro is calibrated.
                while (robot.gyro.isCalibrating())  {
                    Thread.sleep(50);
                    idle();
                }
                */

                timer.reset();
                robot.imu1.initialize(robot.imu1.getParameters());

                telemetry.addData("Mode", "calibrating...");
                telemetry.update();
                // make sure the imu gyro is calibrated before continuing.
                while (!isStopRequested() && !robot.imu1.isGyroCalibrated()) {
                    sleep(50);
                    idle();
                }
                telemetry.addData("Mode", "done, status: " + robot.imu1.getCalibrationStatus().toString() + " | timer: %.2f", timer.milliseconds());
                telemetry.update();
            }

            // Gamepad 1 - Y
            if (gamepad1.y) {
                gyroHold(0.35, 45.0, 2.0);
            }
            // Gamepad 1 - B
            if (gamepad1.b) {
                gyroHold(0.35, 135.0, 2.0);
            }
            // Gamepad 1 - A
            if (gamepad1.a) {
                gyroHold(0.35, -135.0, 2.0);
            }
            // Gamepad 1 - X
            if (gamepad1.x) {
                gyroHold(0.35, -45.0, 2.0);
            }

            composeTelemetry();
            telemetry.update();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        angles1 = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        angles2 = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading()
//                + " | 360 - Heading: " + ((360 - robot.gyro.getHeading()) % 360));
//        telemetry.addData("IMU", "H1: " + formatAngle(angles1.angleUnit, angles1.firstAngle)
//                + " | G1: " + formatAngle(angles1.angleUnit, ((360 - angles1.firstAngle) % 360))
//                + " | H2: " + formatAngle(angles2.angleUnit, angles2.firstAngle)
//                + " | G2: " + formatAngle(angles2.angleUnit, ((360 - angles2.firstAngle) % 360)));
        telemetry.addData("IMU", "H1: %.1f | G1: %.1f"
                , AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle)
                , getRobotHeading());
        telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)"
                , powerFrontLeft, powerFrontRight, powerRearLeft, powerRearRight);
        telemetry.addData("Landing", "Pwr: (%.2f) | Pos: " + robot.motorLanding.getCurrentPosition()
                + " | Limit: " + (robot.limitLanding.getState() == LIMIT_MAG_ON ? "ON" : "OFF"), robot.motorLanding.getPower());
        telemetry.addData("Lift", "Pos: " + robot.motorLift.getCurrentPosition()
                + " | Limit Btm: " + (robot.limitLiftBottom.getState() == LIMIT_MEC_ON ? "ON" : "OFF")
                + " | Limit Top: " + (robot.limitLiftTop.getState() == LIMIT_MEC_ON ? "ON" : "OFF"));
        telemetry.addData("Extend", "Pos: " + robot.motorExtend.getCurrentPosition()
                + " | Lmt: " + (robot.limitExtend.getState() == LIMIT_MAG_ON ? "ON" : "OFF")
                + " | Pwr: %.1f | Scrn %.1f", robot.motorExtend.getPower(), robot.servoScreen.getPosition());
        telemetry.addData("Sensor", "ClrB: %.1f | DstB: %.1f | ClrF: " + (isGold(robot.sensorColorFront) ? "T" : "F")
                        + "%.1f | DstF: %.1f", getColorHue(robot.sensorColorBottom), robot.sensorDistBottom.getDistance(DistanceUnit.CM)
                , getColorHue(robot.sensorColorFront), robot.sensorDistFront.getDistance(DistanceUnit.CM));
        telemetry.addData("Range", "Left: " + robot.rangeLeft.getDistance(DistanceUnit.CM)
                + " | Rear: " + robot.rangeRear.getDistance(DistanceUnit.CM)
                + " | Marker: " + robot.servoMarker.getPosition());
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}


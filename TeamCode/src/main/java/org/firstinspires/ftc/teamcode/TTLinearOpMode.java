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

import java.util.List;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class TTLinearOpMode extends LinearOpMode {

    static final double     HEADING_THRESHOLD       = 1;       // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.15;    // Larger is more responsive, but also less stable

    public static final boolean LIMIT_MAG_ON = false;
    public static final boolean LIMIT_MAG_OFF = true;
    public static final boolean LIMIT_MEC_ON = true;
    public static final boolean LIMIT_MEC_OFF = false;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    static final double SCALE_FACTOR = 255;

    public TTHardware robot = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    private static final String VUFORIA_KEY = "Afr2UsD/////AAAAGVsbt/Ka6EOAj/MfHaZcCWKIcaxclUoOSaQnk/mNz2rzlo+lylAA/E62EFXpjco7vFmqzFyTw+tvPHZj9qMjMKdWHxsHbq/cbQZ7r6BCe5qBzRNVMg69lJEP7dJ+ss5q41SR0Cqs93RSa09U2idgrO5mIsk5VWR19iAbrcuO7cgqfgYQlpeHchR3Z+NdQo/lbWaRqV1fRNVFcwfI8DzLDwvFlEsKass4F5tglt3lDS1zyA/pzfOU9W5zc3OH33dNTN/M4w4dkVDDEOnzmkOP+0svMgM0J4vDaRx+2ZEiFrmVWvW5wz/VumSfniVgg2SlWjy3d0+GCVfTra7OhmBKlWH0qlTYemBc8YE+XtErUG/D";
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;


    public synchronized void waitForStart(Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("in waitForStart()", "runtime: " + runtime.seconds());
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                int Pos = (int) recognition.getLeft();
                                telemetry.addData("Pos", "Pos: " + Pos + " | Label: " + recognition.getLabel());

                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                    }
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    public void timeDrive ( double speed,
                            double time,
                            double angle) {

        ElapsedTime driveTime = new ElapsedTime();

        double robotHeadingRad = 0.0;
        double angleRad = Math.toRadians(angle);
        double powerCompY = 0.0;
        double powerCompX = 0.0;

        double  frontLeftSpeed;
        double  frontRightSpeed;
        double  rearLeftSpeed;
        double  rearRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            driveTime.reset();

            speed = Range.clip(speed, 0.0, 1.0);
            robotHeadingRad = Math.toRadians(360 - robot.gyro.getHeading());
            powerCompY = (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
            powerCompX = -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

            frontLeftSpeed = powerCompY + powerCompX;
            frontRightSpeed = -powerCompY + powerCompX;
            rearLeftSpeed = powerCompY - powerCompX;
            rearRightSpeed = -powerCompY - powerCompX;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && driveTime.seconds() < time) {
                robot.motorFrontLeft.setPower(frontLeftSpeed);
                robot.motorFrontRight.setPower(frontRightSpeed);
                robot.motorRearLeft.setPower(rearLeftSpeed);
                robot.motorRearRight.setPower(rearRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
                telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " + robot.gyro.getIntegratedZValue());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorRearRight.setPower(0);
        }
    }

    public void driveToLine ( double speed,
                            double angle,
                            double time) {

        ElapsedTime driveTime = new ElapsedTime();

        double robotHeadingRad = 0.0;
        double angleRad = Math.toRadians(angle);
        double powerCompY = 0.0;
        double powerCompX = 0.0;

        double  frontLeftSpeed;
        double  frontRightSpeed;
        double  rearLeftSpeed;
        double  rearRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            driveTime.reset();

            speed = Range.clip(speed, 0.0, 1.0);
            robotHeadingRad = Math.toRadians(360 - robot.gyro.getHeading());
            powerCompY = (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
            powerCompX = -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

            frontLeftSpeed = powerCompY + powerCompX;
            frontRightSpeed = -powerCompY + powerCompX;
            rearLeftSpeed = powerCompY - powerCompX;
            rearRightSpeed = -powerCompY - powerCompX;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && getColorHue(robot.sensorColorBottom) < 180.0 && driveTime.seconds() < time) {
                robot.motorFrontLeft.setPower(frontLeftSpeed);
                robot.motorFrontRight.setPower(frontRightSpeed);
                robot.motorRearLeft.setPower(rearLeftSpeed);
                robot.motorRearRight.setPower(rearRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
                telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " + robot.gyro.getIntegratedZValue());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorRearRight.setPower(0);
        }
    }

    public void distLeftDrive ( double speed,
                                double dist,
                                double angle) {

        ElapsedTime driveTime = new ElapsedTime();

        double robotHeadingRad = 0.0;
        double angleRad = Math.toRadians(angle);
        double powerCompY = 0.0;
        double powerCompX = 0.0;

        double  frontLeftSpeed;
        double  frontRightSpeed;
        double  rearLeftSpeed;
        double  rearRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            driveTime.reset();

            speed = Range.clip(speed, 0.0, 1.0);
            robotHeadingRad = Math.toRadians(360 - robot.gyro.getHeading());
            powerCompY = (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
            powerCompX = -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

            frontLeftSpeed = powerCompY + powerCompX;
            frontRightSpeed = -powerCompY + powerCompX;
            rearLeftSpeed = powerCompY - powerCompX;
            rearRightSpeed = -powerCompY - powerCompX;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && driveTime.seconds() < 10.0) {

                /*
                robot.motorFrontLeft.setPower(frontLeftSpeed);
                robot.motorFrontRight.setPower(frontRightSpeed);
                robot.motorRearLeft.setPower(rearLeftSpeed);
                robot.motorRearRight.setPower(rearRightSpeed);
                */

                // Display drive status for the driver.
                telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
                telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " + robot.gyro.getIntegratedZValue());
                telemetry.addData("Left", "Range: " + robot.rangeLeft.getDistance(DistanceUnit.CM));
                telemetry.update();
                sleep(500);
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorRearRight.setPower(0);
        }
    }


    public void distRearDrive ( double speed,
                                double dist,
                                double angle) {

        ElapsedTime driveTime = new ElapsedTime();

        double robotHeadingRad = 0.0;
        double angleRad = Math.toRadians(angle);
        double powerCompY = 0.0;
        double powerCompX = 0.0;

        double  frontLeftSpeed;
        double  frontRightSpeed;
        double  rearLeftSpeed;
        double  rearRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            driveTime.reset();

            speed = Range.clip(speed, 0.0, 1.0);
            robotHeadingRad = Math.toRadians(360 - robot.gyro.getHeading());
            powerCompY = (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
            powerCompX = -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

            frontLeftSpeed = powerCompY + powerCompX;
            frontRightSpeed = -powerCompY + powerCompX;
            rearLeftSpeed = powerCompY - powerCompX;
            rearRightSpeed = -powerCompY - powerCompX;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && robot.rangeRear.getDistance(DistanceUnit.CM) > dist && driveTime.seconds() < 3.0) {
                robot.motorFrontLeft.setPower(frontLeftSpeed);
                robot.motorFrontRight.setPower(frontRightSpeed);
                robot.motorRearLeft.setPower(rearLeftSpeed);
                robot.motorRearRight.setPower(rearRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
                telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " + robot.gyro.getIntegratedZValue());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorRearRight.setPower(0);
        }
    }


    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(0.0);
        robot.motorRearLeft.setPower(0.0);
        robot.motorRearRight.setPower(0.0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double turnSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            turnSpeed  = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            turnSpeed = speed * steer;
        }

        // Send desired speeds to motors.
        robot.motorFrontLeft.setPower(turnSpeed);
        robot.motorFrontRight.setPower(turnSpeed);
        robot.motorRearLeft.setPower(turnSpeed);
        robot.motorRearRight.setPower(turnSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f", turnSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - (360 - robot.gyro.getHeading());
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    // Check if Red
    public boolean isRed(ModernRoboticsI2cColorSensor cs) {
        int redValue = cs.red();
        int greenValue = cs.green();
        int blueValue = cs.blue();
        return redValue > blueValue && redValue > greenValue;
    }

    // Check if Blue
    public boolean isBlue(ModernRoboticsI2cColorSensor cs) {
        int redValue = cs.red();
        int greenValue = cs.green();
        int blueValue = cs.blue();
        return blueValue > redValue && blueValue > greenValue;
    }

    // Check if Gold
    public boolean isGold(ColorSensor cs) {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (cs.red() * SCALE_FACTOR),
                (int) (cs.green() * SCALE_FACTOR),
                (int) (cs.blue() * SCALE_FACTOR),
                hsvValues);

        return (hsvValues[0] < 100 && hsvValues[1] > 0.20);
    }

    public float getColorHue(ColorSensor cs) {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (cs.red() * SCALE_FACTOR),
                (int) (cs.green() * SCALE_FACTOR),
                (int) (cs.blue() * SCALE_FACTOR),
                hsvValues);

        return hsvValues[0];
    }


    // For lower power input, use step function to smooth driving
    double stepInput(double dVal)  {
        double stepVal = 0.0;

        if (gamepad1.right_trigger != 0) {
            stepVal = dVal;
        } else {
            double[] stepArray = {0.0, 0.25, 0.25, 0.25, 0.25, 0.25, 0.33, 0.44, 0.56, 0.68, 0.80};

            // get the corresponding index for the scaleInput array.
            int index = Math.abs((int) (dVal * 10.0));

            // index cannot exceed size of array minus 1.
            if (index > 10) {
                index = 10;
            }

            // get value from the array.
            if (dVal < 0) {
                stepVal = -stepArray[index];
            } else {
                stepVal = stepArray[index];
            }
        }

        // return scaled value.
        return stepVal;
    }

    // For lower power input, use step function to smooth driving
    double stepInputRotate(double dVal)  {
        double stepVal = 0.0;

        if (gamepad1.right_trigger != 0) {
            stepVal = dVal;
        } else {
            double[] stepArray = {0.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

            // get the corresponding index for the scaleInput array.
            int index = Math.abs((int) (dVal * 10.0));

            // index cannot exceed size of array minus 1.
            if (index > 10) {
                index = 10;
            }

            // get value from the array.
            if (dVal < 0) {
                stepVal = -stepArray[index];
            } else {
                stepVal = stepArray[index];
            }
        }

        // return scaled value.
        return stepVal;
    }

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    // private TFObjectDetector tfod;
    public String senseMineral (){

        String goldPosition = "M";
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            /*telemetry.addData("gMX", goldMineralX);
                            telemetry.update();
                            sleep(5000);*/
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                              /*telemetry.addData("sM1X", goldMineralX);
                              telemetry.update();
                              sleep(5000);*/
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                              /*telemetry.addData("sM2X", goldMineralX);
                              telemetry.update();
                              sleep(5000);*/
                            }
                        }
                        //Comparing the left postion to determin the Gold positon
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                //telemetry.addData("Gold Mineral Position", "Left");
                                goldPosition = "L";
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                //telemetry.addData("Gold Mineral Position", "Right");
                                goldPosition = "R";
                            }
                        }

                    }

                }
            }
        }

        return goldPosition;
    }
}


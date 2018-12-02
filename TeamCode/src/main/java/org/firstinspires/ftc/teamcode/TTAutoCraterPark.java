package org.firstinspires.ftc.teamcode;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous(name = "TT Auto Crater Park", group = "TT")
public class TTAutoCraterPark extends TTLinearOpMode {

    // States
    private enum AutoState {
        INITIALIZE,

        LANDING,
        LINE_UP_GOLD,
        DETECT_MINERAL_MID,
        DETECT_MINERAL_RIGHT,
        PUSH_PREP,

        PUSH_GOLD,
        PLACE_TEAM_MARKER,
        MOVE_TO_CRATER,

        STOP
    }
    private AutoState currentState = AutoState.INITIALIZE;

    private GoldMineralPos goldMineralPos = GoldMineralPos.UNKNOWN;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot = new TTHardware();
        robot.init(hardwareMap);

        robot.motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLanding.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLanding.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        sleep (2000);

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();

        robot.gyro.calibrate();
        // make sure the gyro is calibrated.
        while (robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();
        */

        /* activate Tensor Flow Object Detection */
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            List<Recognition> updatedRecognitions = null;
            while (!opModeIsActive() && !isStopRequested()) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    goldMineralPos = getGoldPosition(updatedRecognitions);
                    telemetry.addData("Init Gold Pos", goldMineralPos);
                    telemetry.update();
                }
            }
        }

        waitForStart();

        if (tfod != null) {
            tfod.deactivate();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            switch (currentState) {
                case INITIALIZE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();

                    if (goldMineralPos.equals(GoldMineralPos.UNKNOWN) && tfod != null) {
                        tfod.activate();
                    }

                    currentState = AutoState.LANDING;
                    break;

                case LANDING:
                    telemetry.addData("state", currentState.toString());

                    // Lower landing motor
                    robot.motorLanding.setPower(-0.5);
                    robot.motorLift.setPower(-0.5);
                    sleep(500);
                    robot.motorLift.setPower(0.0);
                    while (robot.limitLanding.getState() == LIMIT_MAG_OFF && runtime.seconds() < 3.0) {
                        robot.motorLanding.setPower(-0.5);
                    }
                    robot.motorLanding.setPower(0.0);

                    // if gold position still unknown, detect for another 5 seconds
                    if (goldMineralPos.equals(GoldMineralPos.UNKNOWN) && tfod != null) {
                        timer.reset();
                        List<Recognition> updatedRecognitions = null;
                        while (timer.seconds() < 5.0) {
                            updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                goldMineralPos = getGoldPosition(updatedRecognitions);
                                telemetry.addData("Gold Pos", goldMineralPos);
                                telemetry.update();
                            }
                        }
                    }
                    if (tfod != null) {
                        tfod.shutdown();
                    }

                    // Move clear off of landing area
                    timeDrive(0.5, 0.3, -90.0);

                    driveToLine(0.35, 0.0, 1.0);

                    gyroHold(0.35, 0.0, 1.0);

                    // if gold still unknown, then assume middle
                    if (goldMineralPos.equals(GoldMineralPos.UNKNOWN)) {
                        goldMineralPos = GoldMineralPos.MIDDLE;
                    }
                    currentState = AutoState.LINE_UP_GOLD;
                    break;

                case DETECT_MINERAL_MID:
                    telemetry.addData("state", currentState.toString());

                    robot.motorExtend.setPower(1.0);
                    while (robot.motorExtend.getCurrentPosition() > -4000 && robot.limitExtend.getState() == LIMIT_MAG_OFF) {
                        robot.motorExtend.setPower(0.3);
                    }
                    robot.motorExtend.setPower(0.0);

                    int count = 0;
                    while (count++ < 5 && (Double.isNaN(robot.sensorDistFront.getDistance(DistanceUnit.CM)) || robot.sensorDistFront.getDistance(DistanceUnit.CM) > 25)) {
                        timeDrive(0.35, 0.2, 90.0);
                        telemetry.addData("sensor", "count: " + count + " | dist: " + robot.sensorDistFront.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    // if still Not-a-Number, move on to next mineral
                    if (Double.isNaN(robot.sensorDistFront.getDistance(DistanceUnit.CM))) {
                        currentState = AutoState.DETECT_MINERAL_RIGHT;
                        break;
                    }

                    if (isGold(robot.sensorColorFront)) {
                        telemetry.addData("Found ", "GOLD");
                        goldMineralPos = GoldMineralPos.MIDDLE;
                        currentState = AutoState.PUSH_PREP;
                        break;
                    } else {
                        telemetry.addData("Found ", "SILVER");
                        currentState = AutoState.DETECT_MINERAL_RIGHT;
                        break;
                    }

                case DETECT_MINERAL_RIGHT:
                    telemetry.addData("state", currentState.toString());

                    timeDrive(0.5, 1.5, 90.0);

                    int countR = 0;
                    while (countR++ < 5 && (Double.isNaN(robot.sensorDistFront.getDistance(DistanceUnit.CM)) || robot.sensorDistFront.getDistance(DistanceUnit.CM) > 25)) {
                        timeDrive(0.35, 0.2, 90.0);
                        telemetry.addData("sensor", "countR: " + countR + " | dist: " + robot.sensorDistFront.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }

                    if (isGold(robot.sensorColorFront)) {
                        telemetry.addData("Found ", "R GOLD");
                        goldMineralPos = GoldMineralPos.RIGHT;
                    } else {
                        telemetry.addData("Found ", "R SILVER");
                        goldMineralPos = GoldMineralPos.LEFT;
                        timeDrive(0.5, 2.5, -88.0);
                    }
                    currentState = AutoState.PUSH_PREP;
                    break;

                case PUSH_PREP:
                    telemetry.addData("state", currentState.toString());

                    while (robot.motorExtend.getCurrentPosition() < -500 && robot.limitExtend.getState() == LIMIT_MAG_OFF) {
                        robot.motorExtend.setPower(-0.3);
                    }
                    robot.motorExtend.setPower(0.0);
//                    robot.servoScreen.setPosition(0.8);

                    timeDrive(0.5, 0.3, 90.0);

                    currentState = AutoState.PUSH_GOLD;
                    break;

                case LINE_UP_GOLD:
                    telemetry.addData("state", currentState.toString());

                    if (goldMineralPos.equals(GoldMineralPos.LEFT)) {
                        timeDrive(0.5, 0.5, -40.0);
                    } else if (goldMineralPos.equals(GoldMineralPos.RIGHT)) {
                        timeDrive(0.5, 1.5, 80.0);
                    } else {
                        timeDrive(0.5, 0.6, 65.0);
                    }
                    gyroHold(0.35, 0.0, 1.0);

                    currentState = AutoState.PUSH_GOLD;
                    break;

                case PUSH_GOLD:
                    telemetry.addData("state", currentState.toString());

                    // Push gold mineral to base (push to left-half of base)
                    timeDrive(0.5, 0.5, 0.0);

                    currentState = AutoState.MOVE_TO_CRATER;
                    break;

                case PLACE_TEAM_MARKER:
                    telemetry.addData("state", currentState.toString());

                    gyroHold(0.35, 45.0, 2.0);

                    if (Double.isNaN(robot.rangeLeft.getDistance(DistanceUnit.CM)) || robot.rangeLeft.getDistance(DistanceUnit.CM) < 30) {
                        timeDrive(0.35, 0.3, 135.0);
                    } else {
                        timeDrive(0.35, 0.3, -45.0);
                    }

                    driveToLine(0.35, 45.0, 1.0);

                    timeDrive(0.5, 0.2, 45.0);

                    // Set marker servo position down to drop team marker
                    robot.servoMarker.setPosition(1.0);

                    sleep(500);

                    currentState = AutoState.MOVE_TO_CRATER;
                    break;

                case MOVE_TO_CRATER:
                    telemetry.addData("state", currentState.toString());

                    timer.reset();
                    robot.motorExtend.setPower(1.0);
                    sleep(200);
                    while (robot.motorExtend.getCurrentPosition() > -3500 && robot.limitExtend.getState() == LIMIT_MAG_OFF && timer.seconds() < 2.0) {
                        robot.motorExtend.setPower(0.3);
                    }
                    robot.motorExtend.setPower(0.0);

                    currentState = AutoState.STOP;
                    break;

                case STOP:
                    telemetry.addData("state", currentState.toString());

                    stop();
                    break;

                default:
                    telemetry.addData("state", currentState.toString());

                    stop();
                    break;
            }

            telemetry.update();

            sleep(100);
            idle();
        }
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;

public class TTHardware {

    public ModernRoboticsI2cGyro gyro = null;

    /* Public OpMode members. */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorRearLeft = null;
    public DcMotor motorRearRight = null;

    public DcMotor motorExtend = null;
    public DcMotor motorLanding = null;
    public DcMotor motorLift = null;

    public DigitalChannel limitExtend = null;
    public DigitalChannel limitLanding = null;
    public DigitalChannel limitLiftBottom = null;
    public DigitalChannel limitLiftTop = null;

    public Servo servoScreen = null;
    public Servo servoMarker = null;

    public ColorSensor sensorColorBottom;
    public DistanceSensor sensorDistBottom;
    public ColorSensor sensorColorFront;
    public DistanceSensor sensorDistFront;
    public DistanceSensor rangeLeft;
    public DistanceSensor rangeRear;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public TTHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorRearLeft   = hwMap.get(DcMotor.class, "motorRearLeft");
        motorRearRight  = hwMap.get(DcMotor.class, "motorRearRight");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorRearLeft.setPower(0.0);
        motorRearRight.setPower(0.0);

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorExtend  = hwMap.get(DcMotor.class, "motorExtend");
        motorLanding  = hwMap.get(DcMotor.class, "motorLanding");
        motorLift  = hwMap.get(DcMotor.class, "motorLift");

        motorExtend.setDirection(DcMotor.Direction.FORWARD);
        motorLanding.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        motorExtend.setPower(0.0);
        motorLanding.setPower(0.0);
        motorLift.setPower(0.0);

        limitExtend = hwMap.get(DigitalChannel.class, "limitExtend");
        limitLanding = hwMap.get(DigitalChannel.class, "limitLanding");
        limitLiftBottom = hwMap.get(DigitalChannel.class, "limitLiftBottom");
        limitLiftTop = hwMap.get(DigitalChannel.class, "limitLiftTop");
        // set the digital channel to input.
        limitExtend.setMode(DigitalChannel.Mode.INPUT);
        limitLanding.setMode(DigitalChannel.Mode.INPUT);
        limitLiftBottom.setMode(DigitalChannel.Mode.INPUT);
        limitLiftTop.setMode(DigitalChannel.Mode.INPUT);

        servoScreen = hwMap.get(Servo.class, "servoScreen");
        servoScreen.setPosition(0.3);

        servoMarker = hwMap.get(Servo.class, "servoMarker");
        servoMarker.setPosition(0.5);

        // get a reference to the color sensor.
        sensorColorBottom = hwMap.get(ColorSensor.class, "sensorColorBottom");
        sensorColorFront = hwMap.get(ColorSensor.class, "sensorColorFront");

        // get a reference to the distance sensor that shares the same name.
        sensorDistBottom = hwMap.get(DistanceSensor .class, "sensorColorBottom");
        sensorDistFront = hwMap.get(DistanceSensor .class, "sensorColorFront");

        rangeLeft = hwMap.get(DistanceSensor .class, "rangeLeft");
        rangeRear = hwMap.get(DistanceSensor .class, "rangeRear");
    }

}


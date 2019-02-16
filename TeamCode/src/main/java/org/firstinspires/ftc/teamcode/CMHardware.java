package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CMHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  backLeftDrive = null;
    public DcMotor  backRightDrive = null;
    public DcMotor  arm1 = null;
    public DcMotor  arm2 = null;
    public DcMotor  actuator1 = null;
    public DcMotor  actuator2 = null;
    public BNO055IMU imu = null;
    public BNO055IMU imu2 = null;

    public CRServo intake = null;

    public Servo gate = null;
    public Servo intakeTiltLeft = null;
    public Servo intakeTiltRight = null;

    public AnalogInput potentiometer = null;

    public RevBlinkinLedDriver blinkin = null;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public CMHardware(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean initServos, boolean initIMU, boolean useDriveEncoders) {
        hwMap = ahwMap;

        //IMU Params
        if (initIMU) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            //init IMU
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
            parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters2.loggingEnabled = true;
            parameters2.loggingTag = "IMU2";
            parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            //init IMU2
            imu2 = hwMap.get(BNO055IMU.class, "imu2");
            imu2.initialize(parameters);
        }
        //init Motors
        leftDrive  = hwMap.get(DcMotor.class, "front_left");
        rightDrive = hwMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left");
        backRightDrive = hwMap.get(DcMotor.class, "back_right");
        arm1 = hwMap.get(DcMotor.class, "arm1");
        arm2 = hwMap.get(DcMotor.class, "arm2");
        actuator1 = hwMap.get(DcMotor.class, "actuator1");
        actuator2 = hwMap.get(DcMotor.class, "actuator2");

        //init Servos
        intake = hwMap.get(CRServo.class, "intake");
//        intake2 = hwMap.get(CRServo.class, "intake2");
        gate = hwMap.get(Servo.class, "gate");
        intakeTiltLeft = hwMap.get(Servo.class, "left");
        intakeTiltRight = hwMap.get(Servo.class, "right");

        //init Sensors
        potentiometer = hwMap.get(AnalogInput.class, "potentiometer");

        //init blinkin
        blinkin = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
//        blinkin.setPattern(pattern);

        //set motor powers to 0
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        arm1.setPower(0);
        arm2.setPower(0);
        actuator1.setPower(0);
        actuator2.setPower(0);

        //set CR servo powers
        intakeSpeed(0);

        //set motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.FORWARD);
        actuator1.setDirection(DcMotor.Direction.REVERSE);
        actuator2.setDirection(DcMotor.Direction.REVERSE  );

        //set CR servo directions
        intake.setDirection(CRServo.Direction.FORWARD);

        //set motor zero power behaviors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //use encoders?
        if (!useDriveEncoders) {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (useDriveEncoders) {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        actuator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set servo positions
        if (initServos) {
            gatePosition("CLOSED");
            intakePosition(0.5);
        }
    }

    //Control both arm motors together
    public void armPower(double power){
        arm1.setPower(power);
        arm2.setPower(power);
    }
    public void driveMotorPower(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);
    }
    public void intakeSpeed(double power){
        intake.setPower(-power);
    }
    public void gatePosition(String position){
        switch (position){
            case "OPEN":
                gate.setPosition(0.95);
                break;
            case "CLOSE":
                gate.setPosition(0.4);
                break;
        }
    }
    public void intakePosition(double position){
        intakeTiltLeft.setPosition(position);
        intakeTiltRight.setPosition(1 - position);
    }
    public void motorRotate(double speed){
        leftDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(-speed);
        rightDrive.setPower(-speed);
    }
    public void actuatorPower(double speed){
        actuator1.setPower(speed);
        actuator2.setPower(speed);
    }
}


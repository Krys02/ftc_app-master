package org.firstinspires.ftc.teamcode.OpModes;


import com.opencv.checkmatecv.CameraViewDisplay;
import com.opencv.checkmatecv.OpenCV;
import com.opencv.checkmatecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareConfigs.CMHardware;
import org.firstinspires.ftc.teamcode.PID.PIDController;

@Autonomous(name = "Crater Autonomous", group = "Autonomous")
public class CraterAuto extends LinearOpMode {

    private CMHardware robot = new CMHardware();
    private GoldAlignDetector detector;


    private ElapsedTime runtime = new ElapsedTime();

    private static final double WHEEL_DIAMETER_INCHES = 6.0;
    private static final double COUNTS_PER_INCH = (868.4) /
            (WHEEL_DIAMETER_INCHES * 3.14159265358979323);

    // IMU Variables
    private Orientation angles;
    private double prevAngle;
    private double targetHeading;
    private String overflow;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    PIDController pidRotate, pidDrive;

    private String goldPos;

    @Override
    public void runOpMode() {

        teleUpdate("Initializing Electronics");

        robot.init(hardwareMap, false, true, true);
        robot.physicalStop.setPosition(0);
        robot.gatePosition("CLOSE");
        robot.marker.setPosition(0.6);
        while (!isStopRequested() && !robot.imu.isGyroCalibrated() && opModeIsActive()) {
            sleep(50);
            idle();
            teleUpdate("Calibrating IMU");
        }
        if (!isStopRequested()) {
            teleUpdate("Setting up PID Controller");
            pidRotate = new PIDController(.005, 0, 0);

            // Set PID proportional value to produce non-zero correction value when robot veers off
            // straight line. P value controls how sensitive the correction is.
            pidDrive = new PIDController(.05, 0, 0);
            sleep(200);
        }
//        robot.intakePosition(0.05);
        if (!isStopRequested()) {
            detector = new GoldAlignDetector();
//            teleUpdate("Setting up Vuforia");
//            detector.VUFORIA_KEY = "AR5tugH/////AAAAmZWCOAi6Vk4zplw3uCD3PmgKbNphk1fcCZOjab9YzLoNQfafXMjye9dox6cEGiBcmfnWt7eZ8ZOdNUKn10YmfgnbM9ntclwrxuEgimv7R3zusivZOQytR+bzWylYXXovLOi1dsgFu6Rm+rJ95sKy6yn/KkrD5nhhCShwxniD5t5FYkxGC7TY681Tx4jfCdnq4aU7tuzOiPnaG8uhaRElSGelPtIBjmBkSHKE8LxZuza3Aewwm2I/v5p5vPrCmDRJlzkWdCWnFZ4v/jmtZAnpR3lx+zA51ZAd9EGanlLSzKQa5C9BtOr9mfLFBKZNxmORyeWX7btz8prPOCeYQyhwtfn6QjVygV2IrCNh7iyL7838";
//            detector.setCAMERA_FORWARD_DISPLACEMENT(7);
//            detector.setCAMERA_LEFT_DISPLACEMENT(0);
//            detector.setCAMERA_VERTICAL_DISPLACEMENT(2);
        }
        if (!isStopRequested()) {
            teleUpdate("Setting up OpenCV Detector");
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();
            sleep(200);
        }
        if (!isStopRequested()) {
            teleUpdate("Tuning OpenCV Detector");
            detector.alignSize = 300; // How wide (in pixels) is the range in which the gold object will be aligned.
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4;
            sleep(200);
        }
        if (!isStopRequested()) {
            teleUpdate("Tuning OpenCV Scorer");
            detector.areaScoringMethod = OpenCV.AreaScoringMethod.MAX_AREA;
            //detector.perfectAreaScorer.perfectArea = 10000;
            detector.maxAreaScorer.weight = 0.005;
            sleep(200);
        }
        if (!isStopRequested()) {
            teleUpdate("Adjusting OpenCV Scorer");
            detector.ratioScorer.weight = 5; //
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
            sleep(200);
        }

        if (!isStopRequested()) {
            teleUpdate("Enabling OpenCV");
            detector.enable();
//            detector.enableVuMarkDetection();
            sleep(200);
        }


        while (!isStarted() && !isStopRequested()) {
            teleUpdate("Initialization Complete, waiting for start");
        }

        waitForStart();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        if (opModeIsActive() && !isStopRequested()) {
            teleUpdate("Ejecting Stop");
            // DRIVE MOTORS BACK TO LIFT STOP
            robot.armPower(0.5);
            robot.physicalStop.setPosition(1);
            sleep(300);
            robot.armPower(0);
            teleUpdate("Landing");
            robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (robot.potentiometer.getVoltage() > 1.35 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(-0.5);
            }
            robot.armPower(0);
        }

        teleUpdate("Landed");
        resetAngle(); // reset heading
        robot.intakePosition(0.3);
        teleUpdate("Strafing off Latch");
        encoderStrafe(0.5, -5, -5, 5);
        teleUpdate("Driving Forward");
        runtime.reset();
        resetEncoders();
        power = 0.5;
        runToPos(0.3, 620, 3);
        teleUpdate("Lowering Arm");
        runtime.reset();
        while (robot.potentiometer.getVoltage() < 2.075 && !isStopRequested() && opModeIsActive() && runtime.seconds() < 1) {
            robot.armPower(0.5);
        }
        robot.armPower(0);
        teleUpdate("Sampling");
        encoderStrafe(.2, 5, 5, 5);
        globalAngle -= 135;
//        rotate(((13*100)*0)+135, 0.5);
        runtime.reset();
        while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 1.5) {
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            correction = checkDirection();
            robot.leftDrive.setPower(correction);
            robot.backLeftDrive.setPower(correction);
            robot.rightDrive.setPower(-correction);
            robot.backRightDrive.setPower(-correction);
        }
        robot.driveMotorPower(0);
        sleep(200);
        if (detector.getAligned()) {
            goldPos = "RIGHT";
            while (robot.potentiometer.getVoltage() > 1.35 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(-0.5);
            }
            robot.armPower(0);
            telemetry.addData("Gold Mineral Found at ", goldPos);
            resetEncoders();
            runToPos(0.3, 300, 2);
            while (robot.potentiometer.getVoltage() > 0.45 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(-0.8);
            }
            robot.armPower(0);
            robot.intakePosition(0.7);
            sleep(200);
            robot.intakeSpeed(0.9);
            runToPos(0.3, -100, 2);
            sleep(100);
            runToPos(0.3, 0, 2);
            while (robot.potentiometer.getVoltage() < 1.4 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(1);
            }
            robot.intakeSpeed(0);
            robot.armPower(0);
            runtime.reset();
            while (runtime.seconds() < 1.8 && opModeIsActive() && !isStopRequested()) {
                if (isStopRequested()) {
                    break;
                }
                robot.actuator1.setPower(1);
                robot.actuator2.setPower(1);
            }
            robot.actuator1.setPower(0);
            robot.actuator2.setPower(0);
            resetEncoders();
            globalAngle -= 32;
            runtime.reset();
            while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 0.5 ) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                correction = checkDirection();
                robot.leftDrive.setPower(correction);
                robot.backLeftDrive.setPower(correction);
                robot.rightDrive.setPower(-correction);
                robot.backRightDrive.setPower(-correction);
            }
            robot.driveMotorPower(0);
//            runToPos(0.5, 750, 4);
            encoderDrive(0.3, 18, 18, 5);

            robot.driveMotorPower(0);
            robot.intakePosition(0.6);
            robot.gatePosition("OPEN");
            sleep(300);
            while (robot.potentiometer.getVoltage() > 1.54 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(-0.25);
            }
            robot.intakeSpeed(0);
            robot.armPower(0);
            resetEncoders();
            runToPos(0.25, -750, 4);
            robot.intakePosition(0.7);
            runtime.reset();
            while (runtime.seconds() < 1.8 && opModeIsActive() && !isStopRequested()) {
                if (isStopRequested()) {
                    break;
                }
                robot.actuator1.setPower(-1);
                robot.actuator2.setPower(-1);
            }
            robot.actuator1.setPower(0);
            robot.actuator2.setPower(0);
            robot.gatePosition("CLOSE");
//            robot.intakePosition(0.3);
            while (robot.potentiometer.getVoltage() < 2.075 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(1);
            }
            robot.armPower(0);
            resetAngle();
//            globalAngle += 70;
//            runtime.reset();
//            while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 1.2) {
//                telemetry.addData("1 imu heading", lastAngles.firstAngle);
//                telemetry.addData("2 global heading", globalAngle);
//                telemetry.addData("3 correction", correction);
//                correction = checkDirection();
//                robot.leftDrive.setPower(correction);
//                robot.backLeftDrive.setPower(correction);
//                robot.rightDrive.setPower(-correction);
//                robot.backRightDrive.setPower(-correction);
//            }
//            robot.driveMotorPower(0);
            encoderDrive(0.5, 22, -22, 3);
            resetAngle();
            resetEncoders();
//            runToPos(0.5, 1500, 5);
            power = 0.6;
            while (opModeIsActive() && robot.leftDrive.getCurrentPosition() < 1280) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
                correction = checkDirection();
                robot.leftDrive.setPower(power + correction);
                robot.backLeftDrive.setPower(power + correction);
                robot.rightDrive.setPower(power - correction);
                robot.backRightDrive.setPower(power - correction);
            }
            globalAngle -= 30;
            resetEncoders();
            power = 0.6;
            while (opModeIsActive() && robot.leftDrive.getCurrentPosition() < 250) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
                correction = checkDirection();
                robot.leftDrive.setPower(power + correction);
                robot.backLeftDrive.setPower(power + correction);
                robot.rightDrive.setPower(power - correction);
                robot.backRightDrive.setPower(power - correction);
            }
            globalAngle -=10;
            while (opModeIsActive() && robot.leftDrive.getCurrentPosition() < 260) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
                correction = checkDirection();
                robot.leftDrive.setPower(power + correction);
                robot.backLeftDrive.setPower(power + correction);
                robot.rightDrive.setPower(power - correction);
                robot.backRightDrive.setPower(power - correction);
            }
            globalAngle -=15;
            resetEncoders();
            while (opModeIsActive() && robot.leftDrive.getCurrentPosition() < 850) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
                correction = checkDirection();
                robot.leftDrive.setPower(power + correction);
                robot.backLeftDrive.setPower(power + correction);
                robot.rightDrive.setPower(power - correction);
                robot.backRightDrive.setPower(power - correction);
            }
//            encoderStrafe(1, -8, -8, 5);
            robot.marker.setPosition(0);
            encoderDrive(1, -15, 15, 2);
            sleep(500);
//            encoderDrive(1, 15, -15, 2);
//            encoderStrafe(1, 8, 8, 5);
            resetEncoders();
            runtime.reset();
            robot.marker.setPosition(0.7);
            while (opModeIsActive() && robot.leftDrive.getCurrentPosition() > -2000) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
                correction = checkDirection();
                robot.leftDrive.setPower(-power + correction);
                robot.backLeftDrive.setPower(-power + correction);
                robot.rightDrive.setPower(-power - correction);
                robot.backRightDrive.setPower(-power - correction);
                if (robot.potentiometer.getVoltage() > 0.6 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(-0.5);
                } else {
                    robot.armPower(0);
                }
                if (runtime.seconds() < 1) {
                    robot.actuatorPower(1);
                } else {
                    robot.actuatorPower(0);
                }
            }
            robot.actuatorPower(0);
            robot.driveMotorPower(0);
            while (robot.potentiometer.getVoltage() > 0.45 && !isStopRequested() && opModeIsActive()) {
                robot.armPower(-0.5);
            }
        } else

        {
            globalAngle -= 45;
            runtime.reset();
            while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 1.5) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                correction = checkDirection();
                robot.leftDrive.setPower(power + correction);
                robot.backLeftDrive.setPower(power + correction);
                robot.rightDrive.setPower(power - correction);
                robot.backRightDrive.setPower(power - correction);
            }
//            rotate(((13*100)*0)+45, 0.5);
            robot.driveMotorPower(0);
            sleep(300);
            if (detector.getAligned()) {
                goldPos = "CENTER";
                while (robot.potentiometer.getVoltage() > 1.35 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(-0.5);
                }
                //50
                robot.armPower(0);
                resetEncoders();
                runToPos(0.3, 650, 3);
                while (robot.potentiometer.getVoltage() > 0.5 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(-0.5);
                }
                robot.armPower(0);
                robot.intakePosition(0.7);
                sleep(200);
                robot.intakeSpeed(0.9);
                runToPos(0.3, 50, 2);
                sleep(100);

                while (robot.potentiometer.getVoltage() < 1.4 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(1);
                }
                robot.intakeSpeed(0);
                robot.armPower(0);
                runtime.reset();
                while (runtime.seconds() < 1.8 && opModeIsActive() && !isStopRequested()) {
                    if (isStopRequested()) {
                        break;
                    }
                    robot.actuator1.setPower(1);
                    robot.actuator2.setPower(1);
                }
                robot.actuator1.setPower(0);
                robot.actuator2.setPower(0);
                resetEncoders();
                globalAngle += 15;
                runtime.reset();
                while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 1.2) {
                    telemetry.addData("1 imu heading", lastAngles.firstAngle);
                    telemetry.addData("2 global heading", globalAngle);
                    telemetry.addData("3 correction", correction);
                    correction = checkDirection();
                    robot.leftDrive.setPower(correction);
                    robot.backLeftDrive.setPower(correction);
                    robot.rightDrive.setPower(-correction);
                    robot.backRightDrive.setPower(-correction);
                }
                robot.driveMotorPower(0);
//                rotate(((13*100)*0)+45, 0.5);
                runToPos(0.2, 700, 4);

                robot.driveMotorPower(0);
                robot.gatePosition("OPEN");
                sleep(1000);
                resetEncoders();
                encoderDrive(0.3, 15, 15, 2);
                runtime.reset();
                while (runtime.seconds() < 1.8 && opModeIsActive() && !isStopRequested()) {
                    if (isStopRequested()) {
                        break;
                    }
                    robot.actuator1.setPower(-1);
                    robot.actuator2.setPower(-1);
                }
                robot.actuator1.setPower(0);
                robot.actuator2.setPower(0);
                globalAngle -= 110;
                runtime.reset();
                while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 50) {
                    telemetry.addData("1 imu heading", lastAngles.firstAngle);
                    telemetry.addData("2 global heading", globalAngle);
                    telemetry.addData("3 correction", correction);
                    correction = checkDirection();
                    robot.leftDrive.setPower(correction);
                    robot.backLeftDrive.setPower(correction);
                    robot.rightDrive.setPower(-correction);
                    robot.backRightDrive.setPower(-correction);
                }
                robot.driveMotorPower(0);

            } else {
                globalAngle -= 45;
                runtime.reset();
                while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 1) {
                    telemetry.addData("1 imu heading", lastAngles.firstAngle);
                    telemetry.addData("2 global heading", globalAngle);
                    telemetry.addData("3 correction", correction);
                    correction = checkDirection();
                    robot.leftDrive.setPower(correction);
                    robot.backLeftDrive.setPower(correction);
                    robot.rightDrive.setPower(-correction);
                    robot.backRightDrive.setPower(-correction);
                }
                goldPos = "LEFT";
                while (robot.potentiometer.getVoltage() > 1.35 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(-0.5);
                }
                robot.armPower(0);
                telemetry.addData("Gold Mineral Found at ", goldPos);
                resetEncoders();
                runToPos(0.5, 280, 2);
                while (robot.potentiometer.getVoltage() > 0.5 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(-0.5);
                }
                robot.armPower(0);
                robot.intakePosition(0.7);
                sleep(200);
                robot.intakeSpeed(0.9);
                runToPos(0.3, -100, 2);
                sleep(100);
                runToPos(0.3, 0, 2);
                robot.intakePosition(0.3);
                while (robot.potentiometer.getVoltage() < 2.075 && !isStopRequested() && opModeIsActive()) {
                    robot.armPower(1);
                }
                robot.armPower(0);
                robot.intakeSpeed(0);

                if (detector.isFound()) {
                    resetEncoders();
                    runToPos(0.5, -800, 5);
                    runToPos(0.5, 400, 5);
                }

            }
        }
//        while (opModeIsActive() && !isStopRequested()){
//            if (isStopRequested()){
//                break;
//            }
//            if (detector.isFound()){
//                telemetry.addData("Gold Mineral Found at XPos:", detector.getXPosition());
//                telemetry.addData("Alligned?", detector.getAligned());
//            } else {
//                telemetry.addData(">", "Gold Mineral not Found");
//            }
//            telemetry.update();
//        }
//        robot.armPower(0);
//        teleUpdate("Strafing Back to Center");
//        encoderStrafe(1, 8, 8, 5);
//        teleUpdate("Rotating");
//
//        encoderDrive(0.5, 44, -44, 5);
//        encoderStrafe(1, -50, -50, 5);
//
//        runtime.reset();
//        teleUpdate("Finding Gold");
//        while (!detector.isFound() && opModeIsActive() && !isStopRequested() && runtime.seconds() < 3) {
//            robot.backRightDrive.setPower(0.2);
//            robot.backLeftDrive.setPower(-0.2);
//            robot.rightDrive.setPower(-0.2);
//            robot.leftDrive.setPower(0.2);
//        }
//        if (detector.isFound()) {
//            while (!detector.getAligned() && opModeIsActive() && !isStopRequested() && runtime.seconds() < 3) {
//                robot.backRightDrive.setPower(0.2);
//                robot.backLeftDrive.setPower(-0.2);
//                robot.rightDrive.setPower(-0.2);
//                robot.leftDrive.setPower(0.2);
//            }
//        }
//        robot.driveMotorPower(0);
//        teleUpdate("Hitting Gold");
//        encoderDrive(0.5, -14, -14, 3);
//        encoderDrive(0.5, 11.5, 11.5, 3);
//
//        encoderDrive(0.5, -21, 21, 5);
//        globalAngle = 90;
//        runtime.reset();
//        power = 0;
//        while (opModeIsActive() && !isStopRequested()) {
//            correction = checkDirection();
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);
//            telemetry.update();
//
//            robot.leftDrive.setPower(power + correction);
//            robot.backLeftDrive.setPower(power + correction);
//            robot.rightDrive.setPower(power - correction);
//            robot.backRightDrive.setPower(power - correction);
////            while (runtime.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
//                robot.leftDrive.setPower(correction);
//                robot.backLeftDrive.setPower(correction);
//                robot.rightDrive.setPower(-correction);
//                robot.backRightDrive.setPower(-correction);
////            }
////
//            resetAngle();
//            runtime.reset();


//        }


        robot.driveMotorPower(0);

        detector.disable();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                            robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy() && !isStopRequested())) {

                if (isStopRequested() || !opModeIsActive()) {
                    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.driveMotorPower(0);
                }
            }

            // Stop all motion;
            robot.driveMotorPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafe(double speed,
                              double frontInches, double backInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (frontInches * COUNTS_PER_INCH / 2);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (-frontInches * COUNTS_PER_INCH / 2);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int) (-backInches * COUNTS_PER_INCH / 2);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int) (backInches * COUNTS_PER_INCH / 2);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                            robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) && !isStopRequested()) {

                if (isStopRequested() || !opModeIsActive()) {
                    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.driveMotorPower(0);
                }
            }

            // Stop all motion;
            robot.driveMotorPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//    private double adjustedHeading() {
//        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return (angles.firstAngle + 180);
//
//    }

//    private void rotate(double degrees, double speed) { // Function to rotate, positive degrees rotate cw, negative degrees rotate ccw
//        double speedAbs = Math.abs(speed);
//        prevAngle = adjustedHeading();
//        telemetry.addData("Status:", "Rotating");
//        telemetry.addData("Last Heading:", prevAngle);
//        if ((prevAngle + degrees) < 360 && (prevAngle + degrees) >= 0) {
//            targetHeading = prevAngle + degrees;
//            overflow = "NONE";
//        } else if ((prevAngle + degrees) >= 360) {
//            targetHeading = (prevAngle + degrees) - 360;
//            overflow = "UPPER";
//        } else if ((prevAngle + degrees) < 0) {
//            targetHeading = (prevAngle + degrees) + 360;
//            overflow = "LOWER";
//        }
//        telemetry.addData("Target Heading:", targetHeading);
//        telemetry.addData("Rotation Overflow:", overflow);
//        telemetry.update();
//        sleep(50);
//        if (overflow.equals("NONE")) { // No weird calculations needed if no overflow
//            if (adjustedHeading() < targetHeading) { // we need to rotate ccw
//                while (opModeIsActive() && !isStopRequested() && adjustedHeading() < targetHeading) {
//                    robot.leftDrive.setPower(-speedAbs);
//                    robot.backLeftDrive.setPower(-speedAbs);
//                    robot.backRightDrive.setPower(speedAbs);
//                    robot.rightDrive.setPower(speedAbs);
//                    telemetry.addData("Target Heading:", targetHeading);
//                    telemetry.addData("Current Heading:", adjustedHeading());
//                    telemetry.update();
//                }
//            }
//            if (adjustedHeading() > targetHeading) { // we need to rotate cw
//                while (opModeIsActive() && !isStopRequested() && adjustedHeading() < targetHeading) {
//                    robot.leftDrive.setPower(speedAbs);
//                    robot.backLeftDrive.setPower(speedAbs);
//                    robot.backRightDrive.setPower(-speedAbs);
//                    robot.rightDrive.setPower(-speedAbs);
//                    telemetry.addData("Target Heading:", targetHeading);
//                    telemetry.addData("Current Heading:", adjustedHeading());
//                    telemetry.update();
//                }
//            }
//        } else if (overflow.equals("UPPER")) { // Some additional steps are needed to account for the heading reset to 0
//            // We do know that if we went over 360, we are turning clockwise, so we only need one direction
//            while (opModeIsActive() && !isStopRequested() && adjustedHeading() >= prevAngle) {
//                // We need to rotate until the current angle is less than the target, then we know we've passed '0'
//                robot.leftDrive.setPower(speedAbs);
//                robot.backLeftDrive.setPower(speedAbs);
//                robot.backRightDrive.setPower(-speedAbs);
//                robot.rightDrive.setPower(-speedAbs);
//                telemetry.addData("Target Heading:", targetHeading);
//                telemetry.addData("Current Heading:", adjustedHeading());
//                telemetry.update();
//            }
//            while (opModeIsActive() && !isStopRequested() && adjustedHeading() <= targetHeading) {
//                // Now we just need to rotate up to the target
//                robot.leftDrive.setPower(speedAbs);
//                robot.backLeftDrive.setPower(speedAbs);
//                robot.backRightDrive.setPower(-speedAbs);
//                robot.rightDrive.setPower(-speedAbs);
//                telemetry.addData("Target Heading:", targetHeading);
//                telemetry.addData("Current Heading:", adjustedHeading());
//                telemetry.update();
//            }
//        } else if (overflow.equals("LOWER")) { // Some additional steps are needed to account for the heading reset to 0
//            // We do know that if we went below 0, we are turning counter-clockwise, so we only need one direction
//            while (opModeIsActive() && !isStopRequested() && adjustedHeading() <= prevAngle) {
//                // We need to rotate until the current angle is greater than the target, then we know we've passed '0'
//                robot.leftDrive.setPower(-speedAbs);
//                robot.backLeftDrive.setPower(-speedAbs);
//                robot.backRightDrive.setPower(speedAbs);
//                robot.rightDrive.setPower(speedAbs);
//                telemetry.addData("Target Heading:", targetHeading);
//                telemetry.addData("Current Heading:", adjustedHeading());
//                telemetry.update();
//            }
//            while (opModeIsActive() && !isStopRequested() && adjustedHeading() >= targetHeading) {
//                // Now we just need to rotate down to the target
//                robot.leftDrive.setPower(-speedAbs);
//                robot.backLeftDrive.setPower(-speedAbs);
//                robot.backRightDrive.setPower(speedAbs);
//                robot.rightDrive.setPower(speedAbs);
//                telemetry.addData("Target Heading:", targetHeading);
//                telemetry.addData("Current Heading:", adjustedHeading());
//                telemetry.update();
//            }
//        }
//        robot.driveMotorPower(0); // Stop the motors
//
//        while (opModeIsActive()) {
//            telemetry.addData("Target Heading:", targetHeading);
//            telemetry.addData("Current Heading:", adjustedHeading());
//            telemetry.update();
//        }
//        teleUpdate("Rotation Complete");
//    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .015;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is cw - is ccw
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within (tolerance) of target. This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.1, power);
        pidRotate.setTolerance(4);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                robot.leftDrive.setPower(power);
                robot.rightDrive.setPower(-power);
                robot.backLeftDrive.setPower(power);
                robot.backRightDrive.setPower(-power);
                telemetry.addData("Status:", "Rotating");
                telemetry.addData("Power:", power);
                telemetry.update();
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.leftDrive.setPower(-power);
                robot.rightDrive.setPower(power);
                robot.backLeftDrive.setPower(-power);
                robot.backRightDrive.setPower(power);
                telemetry.addData("Status:", "Rotating");
                telemetry.addData("Power:", power);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.leftDrive.setPower(-power);
                robot.rightDrive.setPower(power);
                robot.backLeftDrive.setPower(-power);
                robot.backRightDrive.setPower(power);
                telemetry.addData("Status:", "Rotating");
                telemetry.addData("Power:", power);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.driveMotorPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void teleUpdate(String status) { // Simple abstraction for telemetry updates
        telemetry.addData("Status:", status);
        telemetry.update();
    }

    private void resetEncoders() {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runToPos(double speed, int target, double timeoutS) {
        robot.leftDrive.setTargetPosition(target);
        robot.rightDrive.setTargetPosition(target);
        robot.backLeftDrive.setTargetPosition(target);
        robot.backRightDrive.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftDrive.setPower(Math.abs(speed) + correction);
        robot.rightDrive.setPower(Math.abs(speed) - correction);
        robot.backLeftDrive.setPower(Math.abs(speed) + correction);
        robot.backRightDrive.setPower(Math.abs(speed) - correction);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                        robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy() && !isStopRequested())) {

            correction = checkDirection();
//            correction = 0;

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            if (isStopRequested() || !opModeIsActive()) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveMotorPower(0);
            }
        }
    }

}

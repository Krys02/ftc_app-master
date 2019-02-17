package org.firstinspires.ftc.teamcode;


import com.opencv.checkmatecv.CameraViewDisplay;
import com.opencv.checkmatecv.OpenCV;
import com.opencv.checkmatecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Red Crater Autonomous", group="Autonomous")
public class RedCraterAuto extends LinearOpMode {

    private String autoPath = "CRATER";
    private String teamColor = "RED";
    private RevBlinkinLedDriver.BlinkinPattern defaultBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern secondaryBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern tertiaryBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern off = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private RevBlinkinLedDriver.BlinkinPattern white = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    private CMHardware robot = new CMHardware();
    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    private static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;
    private static final double     COUNTS_PER_INCH         = (868.4) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // IMU Variables
    private Orientation angles;
    private double prevAngle;
    private double targetHeading;
    private String overflow;

    @Override
    public void runOpMode() {

        switch (teamColor){
            case "RED":
                defaultBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.RED;
                secondaryBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                tertiaryBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case "BLUE":
                defaultBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                secondaryBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.AQUA;
                tertiaryBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }

        teleUpdate("Initializing Electronics");
        robot.init(hardwareMap, false, true, true);

        teleUpdate("Testing Primary Blinkin Color");
        robot.blinkin.setPattern(defaultBlinkinColor);
        sleep (500);
        teleUpdate("Testing Secondary Blinkin Color");
        robot.blinkin.setPattern(secondaryBlinkinColor);
        sleep (500);
        teleUpdate("Testing Tertiary Blinkin Color");
        robot.blinkin.setPattern(tertiaryBlinkinColor);
        sleep (500);
        robot.blinkin.setPattern(off);

        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            teleUpdate("Calibrating IMU");
        }
        teleUpdate("Setting up OpenCV Detector");
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        teleUpdate("Tuning OpenCV Detector");
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned.
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4;
        sleep(200);

        teleUpdate("Tuning OpenCV Scorer");
        detector.areaScoringMethod = OpenCV.AreaScoringMethod.MAX_AREA;
        //detector.perfectAreaScorer.perfectArea = 10000;
        detector.maxAreaScorer.weight = 0.005;
        sleep(100);

        teleUpdate("Adjusting OpenCV Scorer");
        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        sleep(100);

        teleUpdate("Enabling OpenCV");
        detector.enable();
        sleep(100);

        while (!isStarted() && !isStopRequested()){
            teleUpdate("Initialization Complete, waiting for start");
        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()){
            teleUpdate("Ejecting Stop");
            robot.blinkin.setPattern(white);
            // DRIVE MOTORS BACK TO EJECT STOP
            robot.armPower(0.5);
            sleep(200);
            robot.armPower(0);
            teleUpdate("Landing");
            robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (robot.potentiometer.getVoltage() > 1.36 && !isStopRequested() && opModeIsActive()){
                robot.armPower(-0.5);
            }
            robot.armPower(0);
        }

        teleUpdate("Landed");

        robot.intakePosition(0.3);
        teleUpdate("Strafing off Latch");
        encoderStrafe(1, -8, -8, 5);
        teleUpdate("Driving Forward");
        encoderDrive(1, 8, 8, 5);

        teleUpdate("Lowering Arm");
        while (robot.potentiometer.getVoltage() < 2.075 && !isStopRequested() && opModeIsActive()){
            robot.armPower(0.5);
        }
        robot.armPower(0);
        teleUpdate("Strafing Back to Center");
        encoderStrafe(1, 8, 8, 5);
        teleUpdate("Driving Forward");
        encoderDrive(1, 6, 6, 5);
        teleUpdate("Rotating");
        while (Math.abs(angles.firstAngle) < 175){
            angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Status","Rotating");
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.update();
            robot.motorRotate(0.2);
        }
        robot.driveMotorPower(0);

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
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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

                if (isStopRequested() || !opModeIsActive()){
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
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    private double adjustedHeading(){
        angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 180);

    }
    private void rotate(double degrees, double speed){
        prevAngle = adjustedHeading();
        telemetry.addData("Status:", "Rotating");
        telemetry.addData("Last Heading:", prevAngle);
        if ((prevAngle + degrees) < 360 && (prevAngle + degrees) >= 0){
            targetHeading = prevAngle + degrees;
            overflow = "NONE";
        } else if ((prevAngle + degrees) >= 360){
            targetHeading = (prevAngle + degrees) - 360;
            overflow = "UPPER";
        } else if ((prevAngle + degrees) < 0){
            targetHeading = (prevAngle + degrees) + 360;
            overflow = "LOWER";
        }
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Rotation Overflow:", overflow);
        telemetry.update();
        sleep(50);
        if (overflow.equals("NONE")) { // No weird calculations needed if no overflow
            if (adjustedHeading() < targetHeading) { // we need to rotate clockwise
                while (opModeIsActive() && !isStopRequested() && adjustedHeading() < targetHeading) {
                    robot.motorRotate(-speed);
                    telemetry.addData("Target Heading:", targetHeading);
                    telemetry.addData("Current Heading:", adjustedHeading());
                    telemetry.update();
                }
            }
            if (adjustedHeading() > targetHeading) { // we need to rotate counterclockwise
                while (opModeIsActive() && !isStopRequested() && adjustedHeading() < targetHeading) {
                    robot.motorRotate(speed);
                    telemetry.addData("Target Heading:", targetHeading);
                    telemetry.addData("Current Heading:", adjustedHeading());
                    telemetry.update();
                }
            }
        }
        if (overflow.equals("UPPER")){

        }



    }

    private void teleUpdate(String status){
        telemetry.addData("Status:", status);
        telemetry.update();
    }

}

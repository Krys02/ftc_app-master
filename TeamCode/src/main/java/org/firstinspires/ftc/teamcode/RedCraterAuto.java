package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Safe Autonomous", group="Autonomous")
public class RedCraterAuto extends LinearOpMode {

    private String autoPath = "CRATER";
    private String teamColor = "RED";
    private RevBlinkinLedDriver.BlinkinPattern defaultBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern secondaryBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern tertiaryBlinkinColor;
    private RevBlinkinLedDriver.BlinkinPattern off = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private RevBlinkinLedDriver.BlinkinPattern white = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    CMHardware robot = new CMHardware();

    private ElapsedTime runtime = new ElapsedTime();

    private static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;
    private static final double     COUNTS_PER_INCH         = (868.4) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

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

        robot.blinkin.setPattern(defaultBlinkinColor);
        sleep (500);
        robot.blinkin.setPattern(secondaryBlinkinColor);
        sleep (500);
        robot.blinkin.setPattern(tertiaryBlinkinColor);
        sleep (500);
        robot.blinkin.setPattern(off);

        telemetry.addData("Status:", "Initializing Electronics");
        telemetry.update();
        robot.init(hardwareMap, false, true, true);

        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.addData("Status:", "Calibrating IMU");
            telemetry.update();
        }

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Status:", "Initialized, ready to run");
            telemetry.update();
        }

        waitForStart();


        if (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status:", "Ejecting Stop");
            telemetry.update();
            robot.blinkin.setPattern(white);
            // DRIVE MOTORS BACK TO EJECT STOP
            robot.armPower(0.5);
            sleep(300);
            robot.armPower(0);
            telemetry.addData("Status:", "Landing");
            telemetry.update();
            robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (robot.potentiometer.getVoltage() < 1.05 && !isStopRequested() && opModeIsActive()){
                robot.armPower(-0.5);
            }
            robot.armPower(0);
        }



    }

    private void waitForSeconds(float seconds){
        int millisecondsToWait = Math.round(seconds*1000);
        sleep(millisecondsToWait);
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
            newBackRightTarget = robot.actuator2.getCurrentPosition() + (int) (backInches * COUNTS_PER_INCH / 2);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.actuator2.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.actuator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.actuator2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                            robot.backLeftDrive.isBusy() && robot.actuator2.isBusy()) && !isStopRequested()) {

                if (isStopRequested() || !opModeIsActive()) {
                    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
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
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        resetAngle();

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;
        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
        robot.backLeftDrive.setPower(leftPower);
        robot.backRightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}
        // turn the motors off.
        robot.driveMotorPower(0);
        sleep(500);
        resetAngle();
    }

}

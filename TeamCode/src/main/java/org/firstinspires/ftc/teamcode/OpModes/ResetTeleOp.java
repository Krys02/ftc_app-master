package org.firstinspires.ftc.teamcode.OpModes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CMHardware;

@TeleOp(name = "Reset TeleOp", group = "TeleOp")

public class ResetTeleOp extends LinearOpMode {

    CMHardware robot = new CMHardware();

    @Override
    public void runOpMode() {
        //motor speeds and multipliers
        double speed;
        double strafeSpeed;
        double rotateSpeed;
        double armSpeed;
        double leftMotorSpeed;
        double rightMotorSpeed;
        double backLeftMotorSpeed;
        double backRightMotorSpeed;
        int slowMode = 0;

        float multiplierSpeed = 1f;

        double intakePower = 0f;

        boolean gateOpen = false;
        boolean armRunningToPos = false;



        boolean onMark = false;
        boolean sampleServoDown = false;
        boolean intakeRunning = false;

        //toggles
        boolean slowModeToggleReady = true;
        boolean gateToggleReady = true;
        boolean sampleServoToggleReady = true;
        boolean actuatorToggle = true;
        boolean armPositionToggle = true;
        boolean tiltToggle = true;
        int intakePos = 0;
        // string stuffs
        String actuatorDirection = "STOP";
        // String teamColor = "BLUE";

        double intakeAbs = 0.5f;

        robot.init(hardwareMap, false, false, true);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status:", "Initialized, ready to run");
            telemetry.update();
        }

        waitForStart();

        robot.gatePosition("CLOSE");

        while (opModeIsActive()) {
            telemetry.addData("Status:", "Running");

            speed = -gamepad1.left_stick_y;
            strafeSpeed = gamepad1.left_stick_x;
            rotateSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

            leftMotorSpeed = ((speed - rotateSpeed + strafeSpeed) * multiplierSpeed);
            rightMotorSpeed = ((speed + rotateSpeed - strafeSpeed) * multiplierSpeed);
            backLeftMotorSpeed = ((speed - rotateSpeed - strafeSpeed) * multiplierSpeed);
            backRightMotorSpeed = ((speed + rotateSpeed + strafeSpeed) * multiplierSpeed);

            robot.leftDrive.setPower(leftMotorSpeed);
            robot.rightDrive.setPower(rightMotorSpeed);
            robot.backLeftDrive.setPower(backLeftMotorSpeed);
            robot.backRightDrive.setPower(backRightMotorSpeed);

            robot.intakePosition(0.05);
            if (gamepad1.dpad_up) {
                actuatorDirection = "FORWARDS";
            }
            if (gamepad1.dpad_down) {
                actuatorDirection = "BACKWARDS";
            }
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                actuatorDirection = "STOP";
            }
            if (actuatorDirection.equals("STOP")) {
                robot.actuator1.setPower(0);
                robot.actuator2.setPower(0);
            }
            if (actuatorDirection.equals("FORWARDS")) {
                robot.actuator1.setPower(1);
                robot.actuator2.setPower(1);
            }
            if (actuatorDirection.equals("BACKWARDS")) {
                robot.actuator1.setPower(-1);
                robot.actuator2.setPower(-1);
            }
            armSpeed = gamepad1.right_stick_y;
            robot.arm1.setPower(armSpeed * multiplierSpeed);
            robot.arm2.setPower(armSpeed * multiplierSpeed);

            robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.actuator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.actuator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            telemetry.addData("Potentiometer:", robot.potentiometer.getVoltage());

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLACK);
                }
            });

            telemetry.update();

        }
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}


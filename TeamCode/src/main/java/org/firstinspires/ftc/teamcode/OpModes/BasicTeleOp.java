package org.firstinspires.ftc.teamcode.OpModes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CMHardware;

@TeleOp(name = "Solo TeleOp", group = "TeleOp")

public class BasicTeleOp extends LinearOpMode {

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
        boolean slowMode = false;

        float multiplierSpeed = 1f;

        double intakePower = 0f;

        boolean gateOpen = false;
        boolean armRunningToPos = false;

        boolean armRunningUp = false;
        boolean armRunningDown = false;

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
        boolean gateToggle = true;
        int intakePos = 0;
        boolean stopToggle = true;
        int stopEngaged = 0;
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
        robot.physicalStop.setPosition(1);
        intakePos = 2;
        robot.intakePosition(0.7);

        while (opModeIsActive()) {

            telemetry.addData("Status:", "Running");

            if (!gamepad1.y){
                stopToggle = true;
            }
            if (gamepad1.y && stopToggle){
                stopToggle = false;
                switch (stopEngaged) {
                    case 0:
                        robot.physicalStop.setPosition(1);
                        stopEngaged = 1;
                        break;
                    case 1:
                        robot.physicalStop.setPosition(0);
                        stopEngaged = 0;
                        break;
                }
            }


            speed = -gamepad1.left_stick_y;
            strafeSpeed = gamepad1.left_stick_x;
            rotateSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

            if (gamepad1.b) {
                intakeRunning = true;
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                intakeRunning = false;
            }

            if (!intakeRunning) {
                if (gamepad1.right_bumper) {
                    intakePower = 0.9;
                } else if (gamepad1.left_bumper) {
                    intakePower = -0.9;
                } else {
                    intakePower = 0;
                }
            } else {
                intakePower = 0.9;
            }


            if (gamepad1.left_stick_button && slowModeToggleReady) {
                slowMode = !slowMode;
                slowModeToggleReady = false;
            }
            if (!gamepad1.left_stick_button) {
                slowModeToggleReady = true;
            }

            if ((gamepad1.x || gamepad1.a)) {
                robot.gatePosition("OPEN");
                intakePower = 0.3;
            } else if (!gamepad1.x) {
                robot.gatePosition("CLOSE");
            }
            robot.intakeSpeed(intakePower);

            if (slowMode) {
                multiplierSpeed = 0.5f;
            } else {
                multiplierSpeed = 1f;
            }

            leftMotorSpeed = ((speed - rotateSpeed + strafeSpeed) * multiplierSpeed);
            rightMotorSpeed = ((speed + rotateSpeed - strafeSpeed) * multiplierSpeed);
            backLeftMotorSpeed = ((speed - rotateSpeed - strafeSpeed) * multiplierSpeed);
            backRightMotorSpeed = ((speed + rotateSpeed + strafeSpeed) * multiplierSpeed);

            robot.leftDrive.setPower(leftMotorSpeed);
            robot.rightDrive.setPower(rightMotorSpeed);
            robot.backLeftDrive.setPower(backLeftMotorSpeed);
            robot.backRightDrive.setPower(backRightMotorSpeed);

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

            if (gamepad1.a) {
                robot.intakePosition(0.6);
                intakePos = 2;
            } else {
//                    if (gamepad1.b) {
//                        intakePos = 1;
//                    }
//                if (gamepad1.y) {
//                    intakePos = 2;
//                }
                if (intakePos == 1) {
                    robot.intakePosition(0.3);
                }
                if (intakePos == 2) {
                    robot.intakePosition(0.7);
                }


                if (Math.abs(gamepad1.right_stick_y) != 0) {
                    armRunningToPos = false;
                    armRunningDown = false;
                    armRunningUp = false;
                }
                if (armRunningToPos) {
                    if (armRunningUp) {
                        if (robot.potentiometer.getVoltage() < 1.48) {
                            armSpeed = 1;
                        }
                        if (robot.potentiometer.getVoltage() > 1.48) {
                            armSpeed = 0;
                            armRunningToPos = false;
                            armRunningUp = false;
                        }
                    }
                    if (armRunningDown) {
                        if (robot.potentiometer.getVoltage() > 1.48) {
                            armSpeed = -1;
                        }
                        if (robot.potentiometer.getVoltage() < 1.48) {
                            armSpeed = 0;
                            armRunningToPos = false;
                            armRunningDown = false;
                        }
                    }
                }
                armSpeed = gamepad1.right_stick_y;
                robot.arm1.setPower(armSpeed * multiplierSpeed);
                robot.arm2.setPower(armSpeed * multiplierSpeed);

                robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.actuator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.actuator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.addData("Front Left Drive Encoder", robot.leftDrive.getCurrentPosition());
                telemetry.addData("Front Right Drive Encoder", robot.rightDrive.getCurrentPosition());
                telemetry.addData("Back Left Drive Encoder", robot.backLeftDrive.getCurrentPosition());
                telemetry.addData("Back Right Drive Encoder", robot.backRightDrive.getCurrentPosition());


//                telemetry.addData("Potentiometer:", robot.potentiometer.getVoltage());

                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.BLACK);
                    }
                });

                telemetry.update();

            }

        }
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}


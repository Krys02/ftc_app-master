package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareConfigs.CMHardware;

import java.util.Locale;

@TeleOp(name="Solo TeleOp BLUE", group="TeleOp")

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


        boolean onMark = false;
        boolean sampleServoDown = false;

        //toggles
        boolean slowModeToggleReady = true;
        boolean gateToggleReady = true;
        boolean sampleServoToggleReady = true;
        boolean actuatorToggle = true;
        boolean armPositionToggle = true;
        boolean tiltToggle = true;
        int intakePos = 0;
        //string stuffs
        String actuatorDirection = "STOP";
//        String teamColor = "BLUE";

        robot.init(hardwareMap, false, false, false);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Status:", "Initialized, ready to run");
            telemetry.update();
        }

        waitForStart();

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        robot.gatePosition("CLOSE");

        while (opModeIsActive()) {

//            telemetry.addData("Status:", "Running");

            speed = -gamepad1.left_stick_y;
            strafeSpeed = gamepad1.left_stick_x;
            rotateSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

            if (gamepad1.right_bumper){
                intakePower = 1;
            } else if (gamepad1.left_bumper){
                intakePower = -1;
            } else {
                intakePower = 0;
            }



            if (gamepad1.left_stick_button && slowModeToggleReady){
                    slowMode = !slowMode;
                    slowModeToggleReady = false;
            }
            if (!gamepad1.left_stick_button){
                slowModeToggleReady = true;
            }

            if (gamepad1.x) {
                robot.gatePosition("OPEN");
                intakePower = 1;
            } else if (!gamepad1.x){
                robot.gatePosition("CLOSE");
            }

            robot.intakeSpeed(intakePower);

            if (slowMode){
                multiplierSpeed = 0.5f;
               robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else {
                multiplierSpeed = 1f;
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            leftMotorSpeed = ((speed - rotateSpeed + strafeSpeed) * multiplierSpeed);
            rightMotorSpeed = ((speed + rotateSpeed - strafeSpeed) * multiplierSpeed);
            backLeftMotorSpeed = ((speed - rotateSpeed - strafeSpeed) * multiplierSpeed);
            backRightMotorSpeed = ((speed + rotateSpeed + strafeSpeed) * multiplierSpeed);

            robot.leftDrive.setPower(leftMotorSpeed);
            robot.rightDrive.setPower(rightMotorSpeed);
            robot.backLeftDrive.setPower(backLeftMotorSpeed);
            robot.backRightDrive.setPower(backRightMotorSpeed);

            if (gamepad1.dpad_up){
                actuatorDirection = "FORWARDS";
            }
            if (gamepad1.dpad_down){
                actuatorDirection = "BACKWARDS";
            }
            if (!gamepad1.dpad_down && !gamepad1.dpad_up){
                actuatorDirection = "STOP";
            }
            if (actuatorDirection.equals("STOP")){
                robot.actuator1.setPower(0);
                robot.actuator2.setPower(0);
            }
            if (actuatorDirection.equals("FORWARDS")){
                robot.actuator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.actuator1.setPower(1);
                robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.actuator2.setPower(1);
            }
            if (actuatorDirection.equals("BACKWARDS")){
                robot.actuator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.actuator1.setPower(-1);
                robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.actuator2.setPower(-1);
            }

                if (gamepad1.a) {
                    robot.intakePosition(0.55);
                } else {
                    if (gamepad1.b){
                        intakePos = 1;
                    }
                    if (gamepad1.y){
                        intakePos = 2;
                    }
                    if (intakePos == 1) {
                        robot.intakePosition(1);
                    }
                    if (intakePos == 2) {
                        robot.intakePosition(0.65);
                    }
                }

//                tiltToggle = false;
//                switch (intakePos){
//                    case "DOWN":
//                        robot.intakePosition(0.4);
//                        intakePos = "UP";
//                        break;
//                    case "UP":
//                        robot.intakePosition(0.7);
//                        intakePos = "DOWN";
//                        break;
//                }
            if (!gamepad1.y){
                tiltToggle = true;
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

//            telemetry.addData("Front Left Drive Encoder", robot.leftDrive.getCurrentPosition());
//            telemetry.addData("Front Right Drive Encoder", robot.rightDrive.getCurrentPosition());
//            telemetry.addData("Back Left Drive Encoder", robot.backLeftDrive.getCurrentPosition());
//            telemetry.addData("Back Right Drive Encoder", robot.backRightDrive.getCurrentPosition());
//            telemetry.addData("Actuator 1 Encoder", robot.actuator1.getCurrentPosition());
//            telemetry.addData("Actuator 2 Encoder", robot.actuator2.getCurrentPosition());

//            telemetry.addData("Potentiometer:", robot.potentiometer.getVoltage());

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

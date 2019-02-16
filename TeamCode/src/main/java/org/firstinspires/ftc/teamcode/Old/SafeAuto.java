package org.firstinspires.ftc.teamcode.Old;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.CMHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

@Autonomous(name="Safe Autonomous", group="Autonomous")
public class SafeAuto extends LinearOpMode {

    private String autoPath = "CRATER";
    private int currentAutoPath = 1;
    private RevBlinkinLedDriver.BlinkinPattern defaultBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private RevBlinkinLedDriver.BlinkinPattern secondaryBlinkinColor = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AR5tugH/////AAAAmZWCOAi6Vk4zplw3uCD3PmgKbNphk1fcCZOjab9YzLoNQfafXMjye9dox6cEGiBcmfnWt7eZ8ZOdNUKn10YmfgnbM9ntclwrxuEgimv7R3zusivZOQytR+bzWylYXXovLOi1dsgFu6Rm+rJ95sKy6yn/KkrD5nhhCShwxniD5t5FYkxGC7TY681Tx4jfCdnq4aU7tuzOiPnaG8uhaRElSGelPtIBjmBkSHKE8LxZuza3Aewwm2I/v5p5vPrCmDRJlzkWdCWnFZ4v/jmtZAnpR3lx+zA51ZAd9EGanlLSzKQa5C9BtOr9mfLFBKZNxmORyeWX7btz8prPOCeYQyhwtfn6QjVygV2IrCNh7iyL7838";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    Orientation lastAngles = new Orientation();
    Orientation lastAngles2 = new Orientation();
    double globalAngle, power = .30, correction;
    private Orientation angles;
    private Orientation angles2;

    CMHardware robot = new CMHardware();

    private ElapsedTime runtime = new ElapsedTime();

    private static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;
    private static final double     COUNTS_PER_INCH         = (868.4) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private String samplePosition = "CENTER";

    private double turnSpeed = 0.25;

    @Override
    public void runOpMode() {

        telemetry.addData("Status:", "Initializing Electronics");
        telemetry.update();
        robot.init(hardwareMap, false, true, false);

        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.addData("Status:", "Calibrating IMU");
            telemetry.update();
        }
        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.actuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Status:", "Initialized, ready to run");
            telemetry.update();
        }
        waitForStart();

        if (tfod != null) {
            tfod.activate();
        }

        if (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status:", "Ejecting Stop");
            telemetry.update();
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

            // DRIVE MOTORS BACK TO EJECT STOP

            robot.armPower(0.5);
            sleep(300);
            robot.armPower(0);
            telemetry.addData("Status:", "Landing");
            telemetry.update();
            robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (robot.potentiometer.getVoltage() < 1.05){
                robot.armPower(-0.5);
            }
            robot.armPower(0);


        }
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        resetAngle();
        robot.leftDrive.setPower(0.7);
        robot.rightDrive.setPower(-0.7);
        robot.backLeftDrive.setPower(-0.7);
        robot.backRightDrive.setPower(0.7);
        sleep(300);
        robot.driveMotorPower(0);
        sleep(100);
        encoderDrive(0.5, 4, 4, 5);
        robot.leftDrive.setPower(-0.7);
        robot.rightDrive.setPower(0.7);
        robot.backLeftDrive.setPower(0.7);
        robot.backRightDrive.setPower(-0.7);
        sleep(300);
        robot.driveMotorPower(0);
        sleep(200);
       robot.driveMotorPower(1);
       sleep(1000);
       robot.driveMotorPower(0);


//        rotate(180, 0.5, true);
//        resetAngle();
//        runtime.reset();
////        while (robot.potentiometer.getVoltage() < 0.6 && runtime.seconds() < 2){
////            robot.armPower(0.5);
////        }
//        robot.armPower(0);
//        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//        CameraDevice.getInstance().setFlashTorchMode(true);
//        boolean isAlligned = false;
//        int buffer = 20;
//        runtime.reset();
//        while (opModeIsActive() && !isAlligned && runtime.seconds() < 10) {
//            if (tfod != null) {
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() >= 1) {
//                        int goldMineralX = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getConfidence() > 0.75) {
//                                goldMineralX = (int) recognition.getLeft();
//                                telemetry.addData("Gold Position:", goldMineralX);
//
//                                if (goldMineralX < ((recognition.getImageWidth() / 2) - buffer)) {
//                                    // MINERAL IS LEFT
//                                    robot.motorRotate(-turnSpeed);
//                                } else if (goldMineralX > ((recognition.getImageWidth() / 2) + buffer)) {
//                                    // MINERAL IS RIGHT
//                                    robot.motorRotate(turnSpeed);
//                                } else {
//                                    robot.driveMotorPower(0);
//                                    isAlligned = true;
//                                    robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                                }
//                            }
//                        }
//                        if (goldMineralX != -1) {
//                            telemetry.addData("Status:", "Gold Detected");
//                        } else {
//                            robot.motorRotate(0.2);
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//        CameraDevice.getInstance().setFlashTorchMode(false);
//        if (autoPath.equals("CRATER")) {
//            sleep(1000);
//            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//            robot.driveMotorPower(-1);
//            sleep(800);
//            robot.driveMotorPower(0);
//        } else if (autoPath.equals("DEPOT")){
//            sleep(1000);
//            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//            robot.driveMotorPower(-0.5);
//            sleep(800);
//            robot.driveMotorPower(0);
//            sleep(100);
//            robot.driveMotorPower(0.5);
//            sleep(800);
//            robot.driveMotorPower(0);
////            rotate(0, 0.5, false);
//            rotate(30, 0.5, false);
//            rotate(90, 0.5, true);
//            robot.rightDrive.setPower(1);
//            robot.backRightDrive.setPower(1);
//            robot.leftDrive.setPower(0.8);
//            robot.backLeftDrive.setPower(0.8);
//            sleep(1000);
//            robot.driveMotorPower(0);
//        }
//
//
////        while (robot.potentiometer.getVoltage() < 1.05){
////            robot.armPower(0.5);
////        }
//
////        robot.armPower(0);
////
////        if (autoPath.equals("DEPOT")) {
////            rotate(180, 0.5, true);
////        }
////
////        encoderStrafe(1, 40, 40, 5);
////        rotate(-45, 0.5, true);
////        encoderStrafe(0.7, -10, -10, 5);
////        waitForSeconds(0.2f);
////        encoderStrafe(0.7, 2, 2, 5);
////        encoderDrive(0.5,30,30,5);
////        robot.intakeSpeed(-1);
////        waitForSeconds(1);
////        encoderDrive(1,-48, -48, 5);
////        robot.intakeSpeed(1);
////
////        robot.intakePosition(0.55);
////
////        while (robot.potentiometer.getVoltage() < 1.4){
////            robot.armPower(-0.5);
////        }
////        robot.armPower(0);
//

        // PROGRAM ENDS HERE
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

        // Ensure that the opmode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && !isStopRequested())) {

                if (isStopRequested() || !opModeIsActive()){
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

    /**
     * Rotate left or right the number of degrees up to 180
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees, double power, boolean reset)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        if (reset && !isStopRequested() && opModeIsActive()) {
            resetAngle();
            sleep(100);
        }

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0 && !isStopRequested() && opModeIsActive())
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0 && !isStopRequested() && opModeIsActive())
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.backLeftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
        robot.backRightDrive.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0 && !isStopRequested() && opModeIsActive())
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Current Angle", getAngle());
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() < -degrees) {
                telemetry.addData("Current Angle", getAngle());
                telemetry.update();
            }
        }
        else {    // left turn.
            while (opModeIsActive() && getAngle() > -degrees) {
                telemetry.addData("Current Angle", getAngle());
                telemetry.update();
            }
        }

        // turn the motors off.
        robot.driveMotorPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
//        resetAngle();
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, XYZ, DEGREES);
//        Orientation angles2 = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, XYZ, DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;
//        lastAngles2 = angles2;

        return globalAngle;
    }
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, XYZ, DEGREES);
//        lastAngles2 = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, XYZ, DEGREES);

        globalAngle = 0;
    }
    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

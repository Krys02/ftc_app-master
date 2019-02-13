/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Examples;

import com.opencv.checkmatecv.CameraViewDisplay;
import com.opencv.checkmatecv.OpenCV;
import com.opencv.checkmatecv.detectors.roverrukus.GoldDetector;
import com.opencv.checkmatecv.filters.CheckmateColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;

/**
 *
 */

@TeleOp(name="Webcam Gold Detector", group="OpenCV")

public class WebcamGoldExample extends OpMode {

    //Webcam object
    WebcamName webcamName;

    // OpenCV detector
    GoldDetector detector;

    @Override
    public void init() {
        telemetry.addData("Status", "OpenCV - Webcam Gold Example");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //Retrieves the webcam from the hardware map

        detector = new GoldDetector(); // Create a Gold Detector

        //Sets the Vuforia license key. ALWAYS SET BEFORE INIT!
        detector.VUFORIA_KEY = "AR5tugH/////AAAAmZWCOAi6Vk4zplw3uCD3PmgKbNphk1fcCZOjab9YzLoNQfafXMjye9dox6cEGiBcmfnWt7eZ8ZOdNUKn10YmfgnbM9ntclwrxuEgimv7R3zusivZOQytR+bzWylYXXovLOi1dsgFu6Rm+rJ95sKy6yn/KkrD5nhhCShwxniD5t5FYkxGC7TY681Tx4jfCdnq4aU7tuzOiPnaG8uhaRElSGelPtIBjmBkSHKE8LxZuza3Aewwm2I/v5p5vPrCmDRJlzkWdCWnFZ4v/jmtZAnpR3lx+zA51ZAd9EGanlLSzKQa5C9BtOr9mfLFBKZNxmORyeWX7btz8prPOCeYQyhwtfn6QjVygV2IrCNh7iyL7838";

        //Inits the detector. Choose which camera to use, and whether to detect VuMarks here
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), OpenCV.CameraMode.WEBCAM, false, webcamName);

        //Sets basic detector settings
        detector.yellowFilter = new CheckmateColorFilter(CheckmateColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = OpenCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        detector.enable();
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }


    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        //Telemetry feed of the found rectangle's location (if one exists)
        telemetry.addData("IsFound: ", detector.isFound());
        Rect rect = detector.getFoundRect();
        if(detector.isFound()) telemetry.addData("Location: ", Integer.toString((int) (rect.x + rect.width*0.5)) + ", " + Integer.toString((int) (rect.y+0.5*rect.height)));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (detector != null) detector.disable(); //Make sure to run this on stop!
    }

}

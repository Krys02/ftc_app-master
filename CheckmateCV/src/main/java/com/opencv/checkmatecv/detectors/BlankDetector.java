package com.opencv.checkmatecv.detectors;

import org.opencv.core.Mat;

public class BlankDetector extends OpenCVDetector {
    @Override
    public Mat process(Mat input) {
        // Process frame
        return input;
    }

    @Override
    public void useDefaults() {
        // Add in your scorers here.
    }
}

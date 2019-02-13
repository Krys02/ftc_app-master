package com.opencv.checkmatecv.detectors.roverrukus;

import com.opencv.checkmatecv.detectors.OpenCVDetector;

import org.opencv.core.Mat;


public class VuMarkDetector extends OpenCVDetector {

    @Override
    public Mat process(Mat rgba) {
        return rgba;
    }
    @Override
    public void useDefaults() {}
}

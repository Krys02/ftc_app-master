package com.opencv.checkmatecv.scoring;

import com.opencv.checkmatecv.math.MathFTC;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class ColorDevScorer extends OpenCVScorer{

    private MatOfDouble std = new MatOfDouble();
    private MatOfDouble mean = new MatOfDouble();

    /**
     * @param input - Input mat
     * @return - Difference from perfect score
     */
    @Override
    public double calculateScore(Mat input) {
        Core.meanStdDev(input, mean, std);
        return MathFTC.mean(std.get(0,0));
    }
}

package com.opencv.checkmatecv.scoring;

import org.opencv.core.Mat;

public abstract class OpenCVScorer {
    public abstract double calculateScore(Mat input);
}

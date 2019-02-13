package com.opencv.checkmatecv.filters;

import org.opencv.core.Mat;

public abstract class GenericCVColorFilter {
    public abstract void process(Mat input, Mat mask);

}

package com.opencv.checkmatecv.detectors;

import com.opencv.checkmatecv.OpenCV;
import com.opencv.checkmatecv.OpenCVPipeline;
import com.opencv.checkmatecv.math.MathFTC;
import com.opencv.checkmatecv.scoring.OpenCVScorer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public abstract class OpenCVDetector extends OpenCVPipeline{

    public abstract Mat process(Mat input);
    public abstract void useDefaults();

    private List<OpenCVScorer> scorers = new ArrayList<>();
    private Size initSize;
    private Size adjustedSize;
    private Mat workingMat = new Mat();
    public double maxDifference = 10;

    public Point cropTLCorner = null; //The top left corner of the image used for processing
    public Point cropBRCorner = null; //The bottom right corner of the image used for processing

    public OpenCV.DetectionSpeed speed = OpenCV.DetectionSpeed.BALANCED;
    public double downscale = 0.5;
    public Size   downscaleResolution = new Size(640, 480);
    public boolean useFixedDownscale = true;
    protected String detectorName = "OpenCV Detector";

    public OpenCVDetector(){

    }

    public void setSpeed(OpenCV.DetectionSpeed speed){
        this.speed = speed;
    }

    public void addScorer(OpenCVScorer newScorer){
        scorers.add(newScorer);
    }

    public double calculateScore(Mat input){
        double totalScore = 0;

        for(OpenCVScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }



    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        initSize = rgba.size();

        if(useFixedDownscale){
            adjustedSize = downscaleResolution;
        }else{
            adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
        }

        rgba.copyTo(workingMat);

        if(workingMat.empty()){
            return rgba;
        }
        Imgproc.resize(workingMat, workingMat,adjustedSize); // Downscale
        workingMat = MathFTC.crop(workingMat, cropTLCorner, cropBRCorner);

        Imgproc.resize(process(workingMat),workingMat,getInitSize()); // Process and scale back to original size for viewing
        //Print Info
        Imgproc.putText(workingMat,"OpenCV " + detectorName + ": " + getAdjustedSize().toString() + " - " + speed.toString() ,new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;
    }

    public Size getInitSize() {
        return initSize;
    }

    public Size getAdjustedSize() {
        return adjustedSize;
    }

    public void setAdjustedSize(Size size) { this.adjustedSize = size; }
}

package com.opencv.checkmatecv;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;

import com.opencv.checkmatecv.math.MathFTC;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * An implementation of Vuforia intended to be cross-compatible with OpenCV
 */

public abstract class CVVuforia extends VuforiaLocalizerImpl {

    DrawViewSource rawView;

    Thread workerThread;
    Bitmap outputImage;
    Bitmap rotatedImage;
    Bitmap bitmap;
    Mat inputMat;
    Mat outMat;
    Mat rotatedMat;
    Mat displayMat;
    public CVVuforia(Parameters parameters) {
        super(parameters);
    }

    /**
     * Sets the raw view. Make sure to run this before start()!
     * @param rawView The raw view to which we should display the camera frame
     */
    public void setRawView(DrawViewSource rawView){
        this.rawView = rawView;
        setFrameQueueCapacity(1);
    }

    /**
     * Starts Vuforia
     */
    public synchronized void start(){

        workerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!workerThread.isInterrupted()){
                    render();
                }
            }
        });
        workerThread.setName("Vuforia Thread");
        workerThread.start();
        Log.d("OpenCV", workerThread.getState().toString());
    }

    /**
     * Enables Vuforia VuMark tracking
     */
    public void enableTrack(){
        startTracker();
    }

    /**
     * Stops Vuforia VuMark tracking
     */
    public void disableTrack() {
        stopTracker();
    }

    public abstract Mat analyzeFrame(Mat rgba, Mat gray);

    /**
     * Analyzes the frame passed to this class using OpenCV
     * @param frame The processed frame to be displayed
     */
    public void anaylzeFrame(Frame frame){
        if(frame != null ){
            bitmap = convertFrameToBitmap(frame);
            inputMat = new Mat(bitmap.getWidth(), bitmap.getHeight(), CvType.CV_8UC1);
            Utils.bitmapToMat(bitmap,inputMat);
            rotatedMat = new Mat();
            Core.flip(inputMat.t(), rotatedMat, 1); //Adjust this line to change the image rotation
            outMat = analyzeFrame(rotatedMat, null);

            if(!outMat.empty() ){

                displayMat = new Mat();
                outMat.copyTo(displayMat);
                rotatedImage = Bitmap.createBitmap(displayMat.width(), displayMat.height(), bitmap.getConfig());
                Utils.matToBitmap(displayMat, rotatedImage);

                //height = <user-chosen width> * original height / original width
                Size newSize = MathFTC.fullscreen(displayMat.size(), new Size(rawView.getWidth(), rawView.getHeight()));
                outputImage =  Bitmap.createScaledBitmap(rotatedImage, (int) newSize.width, (int) newSize.height, false);

                ((Activity) rawView.getContext()).runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        rawView.onFrame(outputImage);
                        rawView.invalidate();
                    }
                });
            }else{
                Log.w("OpenCV", "MAT BITMAP MISMATCH OR EMPTY ERROR");
            }
            inputMat.release();
            rotatedMat.release();
            outMat.release();
            displayMat.release();
        } else{
            Log.d("OpenCV", "No Frame!");
        }
    }

    /**
     * Renders the frame passed to this class through Vuforia
     */
    public void render() {
        if(!getFrameQueue().isEmpty()){
            try {
                anaylzeFrame(getFrameQueue().take());
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else{
            Log.v("OpenCV", "Frame is empty. Que Size: " + getFrameQueueCapacity());
        }

    }

    /**
     * Terminates Vuforia
     */
    public synchronized void stop(){
        close();
        ((Activity) rawView.getContext()).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                workerThread.interrupt();
            }
        });
    }
    public void flashOn(){
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    public void flashOff(){
        CameraDevice.getInstance().setFlashTorchMode(false);
    }
}

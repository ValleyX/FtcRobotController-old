package org.firstinspires.ftc.team2844.dogecv.detectors;

import android.util.Log;

import org.firstinspires.ftc.team2844.dogecv.DogeCV;
import org.firstinspires.ftc.team2844.dogecv.scoring.DogeCVScorer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


/**
 * Created by Victo on 9/10/2018.
 */

public abstract class DogeCVDetector extends OpenCvPipeline {

    public abstract Mat process(Mat input);
    public abstract void useDefaults();

    private List<DogeCVScorer> scorers = new ArrayList<>();
    private Size initSize;
    private Size adjustedSize;
    private Mat workingMat = new Mat();
    public double maxDifference = 10;
    
    protected boolean found = false;
    public boolean isSideways = false;
    public double verticalMax = 1.0;
    public double verticalMin = 0.0; //these two added by Harrison Truscott; represent, in percentage (0.0-1.0), the amount of the image to be considered in scoring

    public DogeCV.DetectionSpeed speed = DogeCV.DetectionSpeed.BALANCED;
    protected String detectorName = "DogeCV Detector";

    private Size size;

    protected enum Stage {
        FINAL_DISPLAY,
        THRESHOLD,
        CONTOURS,
        RAW_IMAGE
    }

    protected Stage stageToRenderToViewport = Stage.FINAL_DISPLAY;
    private Stage[] stages = Stage.values();

    public void setSpeed(DogeCV.DetectionSpeed speed){
        this.speed = speed;
    }
    
    public boolean isDetected(){
        return found;
    }

    public void addScorer(DogeCVScorer newScorer){
        scorers.add(newScorer);
    }

    public double calculateScore(Mat input){
        double totalScore = 0;

        for(DogeCVScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }


    @Override
    public final Mat processFrame(Mat input) {
        size = input.size();

        Log.d("DogeCVDetector", "Input mat size:" + input.size());
        input.copyTo(workingMat);

        if(workingMat.empty()){
            return input;
        }

        workingMat = process(workingMat);

        //Print Info
        Imgproc.putText(workingMat,"DogeCV 2020.1 " + detectorName + ": " + stageToRenderToViewport.toString(), new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;
    }

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    public Size getSize() {
        return size;
    }

    public Size getAdjustedSize() {
        //return adjustedSize;
        return size;
    }

    public void setAdjustedSize(Size size) { this.adjustedSize = size; }
}

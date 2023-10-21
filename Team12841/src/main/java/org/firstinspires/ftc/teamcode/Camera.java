package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    public SkystoneDeterminationPipeline pipeline;//The area you checking on the camera stream
    LinearOpMode opMode_; //This allows it to be used throughout the entire class
    OpenCvWebcam webcam;
    //Pass in LinearOp Mode and opMode
    public Camera(LinearOpMode opMode){
        //Assigns the Camera object on the hardware map
        opMode_ = opMode;
        int cameraMonitorViewId = opMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode_.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode_.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //gives the pipeline an area to be in
        pipeline = new SkystoneDeterminationPipeline(261, 75);
        webcam.setPipeline(pipeline);

        // Timeout for obtaining permission is configurable (keeps things from breaking idk)
        webcam.setMillisecondsPermissionTimeout(2500);

        //tells the camera to go on initialization
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            //if you screw something up
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        //Value to return (where the thing is, Pos is for Position)
        public enum MarkerPos {
            Left,
            Right,
            Center
        }
        public MarkerPos markerPos = MarkerPos.Left; //Gives it a default value, will change



        //Makes the Colors
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The main point where the box will be (top-left corner of the box)
        Point REGION1_TOPLEFT_ANCHOR_POINT;
        Point REGION2_TOPLEFT_ANCHOR_POINT;
        Point REGION3_TOPLEFT_ANCHOR_POINT;

        //gives the width and height for each box (added onto anchorpoint to get the bottom right corner of the box)
        static final int REGION_WIDTH = 55;
        static final int REGION_HEIGHT = 80;

        //point A is the top left, point B is the bottom right
        Point region1_pointA;
        Point region1_pointB;
        Point region2_pointA;
        Point region2_pointB;
        Point region3_pointA;
        Point region3_pointB;

        //Working variables
        Mat region1_Cb;
        Mat region1_Cr;
        Mat region2_Cb;
        Mat region2_Cr;
        Mat region3_Cb;
        Mat region3_Cr;

        Mat Cb = new Mat();
        Mat Cr = new Mat();
        //each of these will be the highest average blue or red for all 3 of the boxes
        int avgBlue;
        int avgRed;
        //makes positions for each color depending on which on is higher
        int redPosition;
        int bluePosition;


        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.MarkerPos position;

        //assigns the placement values to the corners of the box (A=topleft, B=bottomright)
        public SkystoneDeterminationPipeline(int x, int y) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(137, 75);
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(0, 75);
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }


        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(input, Cb, 2);
            Core.extractChannel(input, Cr, 0);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            //checks the red & blue values for the 3 boxes and gives them values (score of how blue or red they are)
            int avgBlue1 = (int) Core.mean(region1_Cb).val[0];
            int avgBlue2 = (int) Core.mean(region2_Cb).val[0];
            int avgBlue3 = (int) Core.mean(region3_Cb).val[0];
            int avgRed1 = (int) Core.mean(region1_Cr).val[0];
            int avgRed2 = (int) Core.mean(region2_Cr).val[0];
            int avgRed3 = (int) Core.mean(region3_Cr).val[0];

            //compares them and picks the most blue
            if (avgBlue1 > avgBlue2 && avgBlue1 > avgBlue3) {
                avgBlue = avgBlue1;
                bluePosition = 0;
            }

            else if (avgBlue2 > avgBlue1 && avgBlue2 > avgBlue3) {
                avgBlue = avgBlue2;
                bluePosition = 1;
            }
            else if (avgBlue3 > avgBlue1 && avgBlue3 > avgBlue2) {
                avgBlue = avgBlue3;
                bluePosition = 2;
            }
            //compares them and picks the most red
            if (avgRed1 > avgRed2 && avgRed1 > avgRed3) {
                avgRed = avgRed1;
                redPosition = 0;
            }
            else if (avgRed2 > avgRed1 && avgRed2 > avgRed3) {
                avgRed = avgRed2;
                redPosition = 1;
            }
            else if (avgRed3 > avgRed1 && avgRed3 > avgRed2) {
                avgRed = avgRed3;
                redPosition = 2;
            }


            //REMEMBER TO PUT IF STATEMENT
            if (avgBlue > avgRed){
                if(bluePosition == 0)
                    markerPos = MarkerPos.Right;
                else if(bluePosition == 1)
                    markerPos = MarkerPos.Center;
                else if(bluePosition == 2)
                    markerPos = MarkerPos.Left;
            }
            else if (avgRed > avgBlue){
                if(redPosition == 0)
                    markerPos = MarkerPos.Right;
                else if(redPosition == 1)
                    markerPos = MarkerPos.Center;
                else if(redPosition == 2)
                    markerPos = MarkerPos.Left;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines




            return input;
        }
        public int getAnalysis() {
            return avgBlue;
        }
    }
    public void closeCamera(){
        opMode_.sleep(5000);
        webcam.setPipeline(null);
        webcam.closeCameraDevice();
        opMode_.sleep(2000);
    }
}

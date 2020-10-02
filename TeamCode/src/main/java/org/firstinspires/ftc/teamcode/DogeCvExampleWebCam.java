package org.firstinspires.ftc.teamcode.Team2844;

import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.dogecvDetectors.BlackRectDetector;
import org.firstinspires.ftc.teamcode.dogecvDetectors.SkyStone2Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

//import org.firstinspires.ftc.teamcode.Team2844.Drivers.ColorDriver;


@Autonomous(name = "DogeCvExampleWebCam", group ="Test")
@Disabled
public class DogeCvExampleWebCam extends LinearOpMode {

    SkyStone2Detector skystoneDetector;
    StoneDetector stoneDetector;
    BlackRectDetector blackRectDetector;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        System.out.println("valley: Starting");
        System.out.println("valley: Starting2");
        //waitForStart();

        skystoneDetector = new SkyStone2Detector();
        stoneDetector = new StoneDetector();
        blackRectDetector = new BlackRectDetector();

        //int leftXLine = 110; // purple
        int leftXLine = 130; // purple
        int rightXLine = 200; // green  was 187

        skystoneDetector.SetRequestedYLine(205);
        skystoneDetector.SetRequestedXRightLine(290);
        //skystoneDetector.SetRequestedXLeftLine(50);
        skystoneDetector.SetRequestedXLeftLine(80);
        skystoneDetector.SetRequestedMidlinesRightLine(leftXLine, rightXLine); // purple, green

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        /*
         * Open the connection to the camera device
         */
        //phoneCam.openCameraDevice();
       webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(skystoneDetector);
//        webcam.setPipeline(skystoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        double distanceMoved;
        int skystone = 0;

        double clawopen = 0.1; //0.0
        double clawclose = 0.45; //0.37 //0.55



        ElapsedTime runtime;
        runtime = new ElapsedTime();

        skystone = 2; //default to middle

        while (!isStarted()) {
            if (skystoneDetector.isDetected()) {
                webcam.setPipeline(null);
                telemetry.addData("Skystone found X Y", "%d %d",
                        skystoneDetector.foundRectangle().x, skystoneDetector.foundRectangle().y);
                int stoneXValue = skystoneDetector.foundRectangle().x;
                if (leftXLine >= stoneXValue) {
                    telemetry.addData("stone found left", stoneXValue);
                    skystone = 0;
                } else if (leftXLine < stoneXValue && stoneXValue < rightXLine) {
                    telemetry.addData("stone found middle", stoneXValue);
                    skystone = 1;
                } else if (stoneXValue >= rightXLine) {
                    telemetry.addData("stone found right", stoneXValue);
                    skystone = 2;
                }
                webcam.setPipeline(skystoneDetector);
            }
            else
            {
                telemetry.addLine("Sky stone not found");
            }
            telemetry.update();
            idle();
        }

        //waitForStart();
        runtime.reset();

        webcam.stopStreaming();



    }


}



package org.firstinspires.ftc.teamcode.dogecvDetectors;

import com.disnodeteam.dogecv.DigitalCamera;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name="SKYSTONE Detect Test", group ="Test")
@Disabled
public class SkyStoneDetectTest extends LinearOpMode {

    //DigitalCamera digitalCamera = new DigitalCamera()
   // SkystoneDetector skystoneDetector;
    SkyStone2Detector skystoneDetector;
    StoneDetector stoneDetector;
    BlackRectDetector blackRectDetector;
   OpenCvCamera phoneCam;
//    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {


        //skystoneDetector = new SkystoneDetector();
        skystoneDetector = new SkyStone2Detector();
        stoneDetector = new StoneDetector();
        blackRectDetector = new BlackRectDetector();

        skystoneDetector.SetRequestedYLine(180);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();
//        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(skystoneDetector);
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
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

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

        waitForStart();

        while(opModeIsActive()) {

            if (skystoneDetector.isDetected())
            {
              //  webcam.setPipeline(null);
                phoneCam.setPipeline(null);
                telemetry.addData("Skystone found X Y", "%d %d",
                        skystoneDetector.foundRectangle().x, skystoneDetector.foundRectangle().y);
            }
            else
            {
                telemetry.addLine("Sky stone not found");
            }
            telemetry.update();
            //webcam.setPipeline(skystoneDetector);
            phoneCam.setPipeline(skystoneDetector);
            idle();
            /*
            if (stoneDetector.isDetected())
            {
                phoneCam.setPipeline(null);
                telemetry.addData("Stone Found", 1);


                for ( Rect rect : stoneDetector.foundRectangles() ) {
                    //System.out.println("rect " + rect.toString());
                    telemetry.addData("Stone found X Y", "%d %d", rect.x, rect.y);
                    telemetry.addData("Stone Found 2", 1);
                }
                phoneCam.setPipeline(stoneDetector);
                idle();
            }
            else
            {
                telemetry.addLine("stone not found");
            }
            telemetry.update();



        */
            /*
            if (blackRectDetector.isDetected())
            {
                //phoneCam.setPipeline(null);
                webcam.setPipeline(null);
                telemetry.addData("black Found", 1);


                for ( Rect rect : blackRectDetector.foundRectangles() ) {
                    //System.out.println("rect " + rect.toString());
                    telemetry.addData("Stone found X Y", "%d %d", rect.x, rect.y);
                    telemetry.addData("Stone Found 2", 1);
                }

               // phoneCam.setPipeline(blackRectDetector);
                webcam.setPipeline(blackRectDetector);
                idle();
            }
            else
            {
                telemetry.addLine("stone not found");
            }
            telemetry.update();
*/

        }


    }
}

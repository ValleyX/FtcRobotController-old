package org.firstinspires.ftc.team12841;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

@Autonomous(name = "test")
public class testCamera extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamLeft; // USB 3.0
        WebcamName webcamRight; // USB 2.0
        OpenCvSwitchableWebcam switchableWebcam;
        webcamLeft = hardwareMap.get(WebcamName.class, "Webcam"); // USB 3.0

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

       // switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamLeft);

        waitForStart();
    }
}

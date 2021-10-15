package org.firstinspires.ftc.teamcode.Team2844.Drivers;
import android.graphics.Region;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
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
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

public class CameraTestingL
{
    LinearOpMode OpMode_;

    public MandoRobotHardware.SkystoneDeterminationPipeline pipeline;
    public WebcamName webcamLeft; //
    public WebcamName webcamRight; //
    public OpenCvSwitchableWebcam switchableWebcam;

    public enum cameraSelection
    {
        LEFT,
        RIGHT
    }
/*
    public CameraTestingL(LinearOpMode opMode, int x, int y, final MandoRobotHardware.cameraSelection camera)
    {
        // Public OpMode members
        OpMode_ = opMode;

        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());
        webcamLeft = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Left"); // USB 3.0
        webcamRight = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new MandoRobotHardware.SkystoneDeterminationPipeline(x, y);

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamRight);
        switchableWebcam.openCameraDevice();
        switchableWebcam.setPipeline(pipeline);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //pick desired camera here
                if (camera == MandoRobotHardware.cameraSelection.LEFT)
                {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcamLeft);
                }
                else
                {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(webcamRight);
                }
            }
        });


    }
*/
}

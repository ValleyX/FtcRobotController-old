package org.firstinspires.ftc.team2844.TestCode;
import android.graphics.Region;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
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

@TeleOp (name = "CameraTestingL")
public class CameraTestingL extends LinearOpMode {
    //LinearOpMode OpMode_;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this,145,120, RobotHardware.cameraSelection.DOWN);

        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;

        while (!isStarted())
        {
            path = robot.pipeline.position;
            //telemetry.addData("AverageMiddle", robot.pipeline.SkystoneAverageMiddle);
            //telemetry.addData("AverageLeft", robot.pipeline.SkystoneAverageLeft);
            //telemetry.addData("AverageRight", robot.pipeline.SkystoneAverageRight);
            //telemetry.addData("Max avg", Math.max(Math.max(robot.pipeline.SkystoneAverageMiddle, robot.pipeline.SkystoneAverageLeft), robot.pipeline.SkystoneAverageRight));
            telemetry.addData("Position", path);
            telemetry.update();
        }
        /** test for actual code
        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Left){

        }

*/



    }
}

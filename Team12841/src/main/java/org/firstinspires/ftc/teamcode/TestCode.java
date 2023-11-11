package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

@TeleOp(name = "TestDrive")
public class TestCode extends LinearOpMode {

    //declares the motors
    public DcMotor lFMotor;
    public DcMotor lBMotor;
    public DcMotor rFMotor;
    public DcMotor rBMotor;

    @Override
    public void runOpMode() throws InterruptedException {
//        Camera camera = new Camera(this);


        lFMotor = hardwareMap.get(DcMotor.class, "lFDrive");
        lBMotor = hardwareMap.get(DcMotor.class, "lBDrive");
        rFMotor = hardwareMap.get(DcMotor.class, "rFDrive");
        rBMotor = hardwareMap.get(DcMotor.class, "rBDrive");
/*
        WebcamName webcamleft;
        OpenCvSwitchableWebcam switchableWebcam;
        webcamleft = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamleft, webcamleft);
        //switchableWebcam.setActiveCamera(webcamleft);
        while (!opModeIsActive())
        {
            telemetry.addData("Cone Color", Camera.SkystoneDeterminationPipeline.MarkerPos.Red);
            telemetry.update();

        }
*/



        double lefty;
        double leftx;
        double righty;


        waitForStart();

        //controls
        while (opModeIsActive()) {
            //makes the variables for the sticks
            lefty = -gamepad1.left_stick_y;
            leftx = gamepad1.left_stick_x;
            righty = -gamepad1.right_stick_y;

            //set the power to the motors
            if (lefty > 0.0) {
                lFMotor.setPower(righty);
                lBMotor.setPower(righty);
                rFMotor.setPower(-righty);
                rBMotor.setPower(-righty);
            }else if (lefty < 0.0){
                lFMotor.setPower(-righty);
                lBMotor.setPower(-righty);
                rFMotor.setPower(righty);
                rBMotor.setPower(righty);
            }
            if (leftx > 0.0){
                lFMotor.setPower(righty);
                lBMotor.setPower(righty);
                rFMotor.setPower(righty);
                rBMotor.setPower(righty);
            } else if (leftx < 0.0){
                rFMotor.setPower(-righty);
                rBMotor.setPower(-righty);
                lFMotor.setPower(-righty);
                lBMotor.setPower(-righty);
            }

        }
    }
}
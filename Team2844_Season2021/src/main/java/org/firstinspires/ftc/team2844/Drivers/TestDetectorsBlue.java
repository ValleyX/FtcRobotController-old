package org.firstinspires.ftc.team2844.Drivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.dogecv.filters.LeviColorFilter;

//@Disabled

@TeleOp(name="TestDetectorsBlue")
public class TestDetectorsBlue extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        RobotHardwareTestDetectors robotHardwareTestDetectors = new RobotHardwareTestDetectors(hardwareMap, this, 0, 0, RobotHardwareTestDetectors.cameraSelection.LEFT);


        Boolean yPressedUp = false;
        Boolean aPressedDown = false;


        robotHardwareTestDetectors.blueAlignPipeline.useDefaults();
        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Is Found", robotHardwareTestDetectors.blueAlignPipeline.isFound());
            telemetry.addData("redTheshold", robotHardwareTestDetectors.blueAlignPipeline.blueTheshold);

            if (robotHardwareTestDetectors.blueAlignPipeline.isFound()) {
                telemetry.addData("Aligned", robotHardwareTestDetectors.blueAlignPipeline.getAligned());
                telemetry.addData("Get X", robotHardwareTestDetectors.blueAlignPipeline.getXPosition());
                telemetry.addData("width", robotHardwareTestDetectors.blueAlignPipeline.getFoundRect().width);
            }
            telemetry.update();

            if ((gamepad1.y) && (!yPressedUp)) {
                robotHardwareTestDetectors.blueAlignPipeline.blueTheshold += 1;
                robotHardwareTestDetectors.blueAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.blueAlignPipeline.blueTheshold);
                yPressedUp = true;
            } else if (!gamepad1.y) {
                yPressedUp = false;
            }

            if ((gamepad1.a)  && (!aPressedDown)) {
                robotHardwareTestDetectors.blueAlignPipeline.blueTheshold -= 1;
                robotHardwareTestDetectors.blueAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.blueAlignPipeline.blueTheshold);

                aPressedDown = true;
            } else if (!gamepad1.a)
            {
                aPressedDown = false;
            }

            if (gamepad1.b) {
               // robotHardwareTestDetectors.switchableWebcam.stopStreaming();
               //nb sleep(4000);
                robotHardwareTestDetectors.switchableWebcam.setPipeline(robotHardwareTestDetectors.blueAlignPipeline);
                //sleep(4000);
                //robotHardwareTestDetectors.switchableWebcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            if (gamepad1.x) {
                robotHardwareTestDetectors.switchableWebcam.setPipeline(robotHardwareTestDetectors.goldAlignPipeline);
                //sleep(10000);
                //robotHardwareTestDetectors.switchableWebcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            if (gamepad1.dpad_right) {
                robotHardwareTestDetectors.switchableWebcam.setPipeline(robotHardwareTestDetectors.pipeline);
                //sleep(4000);

            }
        }
        //waitForStart();




    }
}

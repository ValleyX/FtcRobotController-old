package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Drivers.RobotHardwareTestDetectors;
import org.firstinspires.ftc.team2844.dogecv.filters.LeviColorFilter;

//@Disabled

@TeleOp(name="TestDetectorsRed")
public class TestDetectorsRed extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        RobotHardwareTestDetectors robotHardwareTestDetectors = new RobotHardwareTestDetectors(hardwareMap, this, 0, 0, RobotHardwareTestDetectors.cameraSelection.LEFT);


        Boolean yPressedUp = false;
        Boolean aPressedDown = false;


        robotHardwareTestDetectors.redAlignPipeline.useDefaults();
        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Is Found", robotHardwareTestDetectors.redAlignPipeline.isFound());
            telemetry.addData("redTheshold", robotHardwareTestDetectors.redAlignPipeline.redTheshold);

            if (robotHardwareTestDetectors.redAlignPipeline.isFound()) {
                telemetry.addData("Aligned", robotHardwareTestDetectors.redAlignPipeline.getAligned());
                telemetry.addData("Get X", robotHardwareTestDetectors.redAlignPipeline.getXPosition());
                telemetry.addData("width", robotHardwareTestDetectors.redAlignPipeline.getFoundRect().width);
            }
            telemetry.update();

            if ((gamepad1.y) && (!yPressedUp)) {
                robotHardwareTestDetectors.redAlignPipeline.redTheshold += 1;
                robotHardwareTestDetectors.redAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.redAlignPipeline.redTheshold);
                yPressedUp = true;
            } else if (!gamepad1.y) {
                yPressedUp = false;
            }

            if ((gamepad1.a)  && (!aPressedDown)) {
                robotHardwareTestDetectors.redAlignPipeline.redTheshold -= 1;
                robotHardwareTestDetectors.redAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.redAlignPipeline.redTheshold);

                aPressedDown = true;
            } else if (!gamepad1.a)
            {
                aPressedDown = false;
            }

            if (gamepad1.b) {
               // robotHardwareTestDetectors.switchableWebcam.stopStreaming();
               //nb sleep(4000);
                robotHardwareTestDetectors.switchableWebcam.setPipeline(robotHardwareTestDetectors.redAlignPipeline);
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

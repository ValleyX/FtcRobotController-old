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

        robotHardwareTestDetectors.goldAlignPipeline.useDefaults();
        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Is Found", robotHardwareTestDetectors.redAlignPipeline.isFound());
           // telemetry.addData("yellowTheshold", robotHardwareTestDetectors.redAlignPipeline.yellowTheshold);

            if (robotHardwareTestDetectors.redAlignPipeline.isFound()) {
                telemetry.addData("Aligned", robotHardwareTestDetectors.redAlignPipeline.getAligned());
                telemetry.addData("Get X", robotHardwareTestDetectors.redAlignPipeline.getXPosition());
                telemetry.addData("width", robotHardwareTestDetectors.redAlignPipeline.getFoundRect().width);
            }
            telemetry.update();

            if ((gamepad1.y) && (!yPressedUp)) {
                robotHardwareTestDetectors.redAlignPipeline.redTheshold += 1;
                robotHardwareTestDetectors.redAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold);
                yPressedUp = true;
            } else if (!gamepad1.y) {
                yPressedUp = false;
            }

            if ((gamepad1.a)  && (!aPressedDown)) {
                robotHardwareTestDetectors.redAlignPipeline.redTheshold -= 1;
                robotHardwareTestDetectors.redAlignPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold);

                aPressedDown = true;
            } else if (!gamepad1.a)
            {
                aPressedDown = false;
            }

        }
        //waitForStart();




    }
}

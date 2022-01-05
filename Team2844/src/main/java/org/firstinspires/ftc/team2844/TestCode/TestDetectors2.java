package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.team2844.Drivers.RobotHardwareTestDetectors;
import org.firstinspires.ftc.team2844.TestDrivers.SensorREVColorDistance_CB;
import org.firstinspires.ftc.team2844.dogecv.filters.LeviColorFilter;

import java.nio.file.Watchable;

//@Disabled

@TeleOp(name="TestDetectors2")
public class TestDetectors2 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        RobotHardwareTestDetectors robotHardwareTestDetectors = new RobotHardwareTestDetectors(hardwareMap, this, 0, 0, RobotHardwareTestDetectors.cameraSelection.LEFT);
        Boolean yPressedUp = false;
        Boolean aPressedDown = false;

        robotHardwareTestDetectors.goldAlignPipeline.useDefaults();
        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Is Found", robotHardwareTestDetectors.goldAlignPipeline.isFound());
            telemetry.addData("yellowTheshold", robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold);

            if (robotHardwareTestDetectors.goldAlignPipeline.isFound()) {
                telemetry.addData("Aligned", robotHardwareTestDetectors.goldAlignPipeline.getAligned());
                telemetry.addData("Get X", robotHardwareTestDetectors.goldAlignPipeline.getXPosition());
                telemetry.addData("width", robotHardwareTestDetectors.goldAlignPipeline.getFoundRect().width);
            }
            telemetry.update();
            if ((gamepad1.y) && (!yPressedUp)) {
                robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold += 1;
                robotHardwareTestDetectors.goldAlignPipeline.yellowFilter.updateSettings(LeviColorFilter.ColorPreset.YELLOW,robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold);
                yPressedUp = true;
            } else if (!gamepad1.y) {
                yPressedUp = false;
            }

            if ((gamepad1.a)  && (!aPressedDown)) {
                robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold -= 1;
                robotHardwareTestDetectors.goldAlignPipeline.yellowFilter.updateSettings(LeviColorFilter.ColorPreset.YELLOW,robotHardwareTestDetectors.goldAlignPipeline.yellowTheshold);

                aPressedDown = true;
            } else if (!gamepad1.a)
            {
                aPressedDown = false;
            }

        }
        //waitForStart();




    }
}

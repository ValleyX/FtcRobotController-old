package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.Drivers.RobotHardwareTestDetectors;

@TeleOp(name="TestDetectors")
public class TestDetectors extends LinearOpMode {


   // RobotHardwareTestDetectors robotHardwareTestDetectors = new RobotHardwareTestDetectors(hardwareMap, this, 0, 0, RobotHardwareTestDetectors.cameraSelection.LEFT);
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}

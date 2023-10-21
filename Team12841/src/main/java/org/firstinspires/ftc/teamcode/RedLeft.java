package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedLeft")

public class RedLeft extends LinearOpMode {
    RobotHardware robotHardware;
    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        Camera.SkystoneDeterminationPipeline.MarkerPos markerPosSeen;

        while (!opModeIsActive())
        {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.update();
            markerPosSeen = robotHardware.camera.pipeline.markerPos;
        }

        robotHardware.camera.closeCamera();
        robotHardware.aprilTagCamera.initAprilTag();

        //do work for auto now


        //starts looking for april tag


    }
}

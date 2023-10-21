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

       // robotHardware.camera.closeCamera();
       // robotHardware.aprilTagCamera.initAprilTag();

        //do work for auto now
//        robotHardware.driveStraight(0.3, 12.0, 0.0);
//        sleep(1000);
//        robotHardware.driveStraight(0.3, -12.0, 0.0);
       // sleep(1000);

        robotHardware.turnToHeading(0.3, 90.0);
//        robotHardware.holdHeading(robotHardware.TURN_SPEED, 90.0, 0.5);
//
//        robotHardware.driveStraight(robotHardware.DRIVE_SPEED, 12.0, 90.0);
//        robotHardware.turnToHeading(robotHardware.TURN_SPEED, 180.0);
//        robotHardware.holdHeading(robotHardware.TURN_SPEED, 180.0, 0.5);
//
//        robotHardware.driveStraight(robotHardware.DRIVE_SPEED, 12.0, 180.0);
//        robotHardware.turnToHeading(robotHardware.TURN_SPEED, 270.0);
//        robotHardware.holdHeading(robotHardware.TURN_SPEED, 270.0, 0.5);
//
//        robotHardware.driveStraight(robotHardware.DRIVE_SPEED, 12.0, 270.0);
//        robotHardware.turnToHeading(robotHardware.TURN_SPEED, 360.0);
//        robotHardware.holdHeading(robotHardware.TURN_SPEED, 360.0, 0.5);
//
//        robotHardware.driveStraight(robotHardware.DRIVE_SPEED, 12.0, 360.0);

        //starts looking for april tag


    }
}

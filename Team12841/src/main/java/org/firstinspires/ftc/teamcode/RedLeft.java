package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedLeft")

public class RedLeft extends LinearOpMode {
    RobotHardware robotHardware;
    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.RedL);
//        Camera.SkystoneDeterminationPipeline.MarkerPos markerPosSeen;

        while (!opModeIsActive())
        {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.addData("Blue Pos 1 Color RGB ", robotHardware.camera.pipeline.avgBlue1);
            telemetry.addData("Blue Pos 2 Color RGB ", robotHardware.camera.pipeline.avgBlue2);
            telemetry.addData("Red Pos 1 Color RGB ", robotHardware.camera.pipeline.avgRed1);
            telemetry.addData("Red Pos 2 Color RGB ", robotHardware.camera.pipeline.avgRed2);//no i will keep putting weird comments in your code no
            telemetry.update();
//            markerPosSeen = robotHardware.camera.pipeline.markerPos;
        }

//        markerPosSeen = robotHardware.camera.pipeline.markerPos;
//        String testMarkerPosSeen = "Left";

        robotHardware.camera.closeCamera(); //if ur reading this,s ur gay see; 2nd def.

        if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Left)
        {
            robotHardware.driveStraight(0.5, 5, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.5, -22.0);
            robotHardware.driveStraight(0.5, 12, -22.0);
            robotHardware.driveStraight(0.5, -12, -22.0);
            robotHardware.turnToHeading(0.5, 0.0);
            robotHardware.driveStraight(0.5, 45, 0.0);
            robotHardware.turnToHeading(0.5, 90);
            robotHardware.driveStraight(0.5, 60, 90);
        }
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center)
        {
            robotHardware.driveStraight(0.5, 25, 0.0);
        }


    }
}

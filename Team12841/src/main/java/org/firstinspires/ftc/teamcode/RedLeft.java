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
            robotHardware.driveStraight(0.3, 5, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.3, -22.0);
            robotHardware.driveStraight(0.3, 16, -22.0);
            robotHardware.driveStraight(0.3, -20, -22.0);
            robotHardware.turnToHeading(0.3, 0);

            robotHardware.driveStraight(0.3, 48, 0.0);
            robotHardware.turnToHeading(0.3, 92);
            robotHardware.driveStraight(0.3, 100, 92);
        }
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center)
        {
            robotHardware.driveStraight(0.3, 33.5, 0.0);
            robotHardware.driveStraight(0.3, -13, 0);
            robotHardware.turnToHeading(0.3, -90);
            robotHardware.driveStraight(0.3,13, -90);
            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 29, 0.0);
            robotHardware.turnToHeading(0.3, 92);
            robotHardware.driveStraight(0.3, 120, 92);
        }
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right){
            robotHardware.driveStraight(0.1, 20, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.1, 60);
            robotHardware.driveStraight(0.1, 12, 60);
            robotHardware.driveStraight(0.3, -14, 60);
            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 31, 0.0);
            robotHardware.turnToHeading(0.3, 92);
            robotHardware.driveStraight(0.3, 100, 92);
        }

    }
}

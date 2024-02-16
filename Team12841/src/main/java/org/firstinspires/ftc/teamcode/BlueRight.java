package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "BlueRight")

public class BlueRight extends LinearOpMode {
    RobotHardware robotHardware;
    LiftHardware liftHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.BlueR);
        liftHardware = new LiftHardware(robotHardware, this);

        while (!opModeIsActive()) {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.addData("Blue Pos 1 Color RGB ", robotHardware.camera.pipeline.avgBlue1);
            telemetry.addData("Blue Pos 2 Color RGB ", robotHardware.camera.pipeline.avgBlue2);
            telemetry.addData("Red Pos 1 Color RGB ", robotHardware.camera.pipeline.avgRed1);
            telemetry.addData("Red Pos 2 Color RGB ", robotHardware.camera.pipeline.avgRed2);//no i will keep putting weird comments in your code no
            telemetry.update();
//            markerPosSeen = robotHardware.camera.pipeline.markerPos;
        }
        robotHardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
//        markerPosSeen = robotHardware.camera.pipeline.markerPos;
//        String testMarkerPosSeen = "Left";

        robotHardware.camera.closeCamera(); //if ur reading this,s ur gay see; 2nd def.

        if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right) {
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 5, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.3, 25.0);
            robotHardware.driveStraight(0.3, 18, 26.0);
            robotHardware.driveStraight(0.3, -19, 26.0); //places

            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 46, 0.0);
            robotHardware.turnToHeading(0.3, -92);
            sleep(2000);
            robotHardware.driveStraight(0.3, 80, -92);

            robotHardware.turnToHeading(0.3, -180.0);
            robotHardware.driveStraight(0.3, 19.5, -180);
            robotHardware.turnToHeading(0.3, -94.0);
            robotHardware.driveStraight(0.1, 16.0, -94);

            liftHardware.moveElbow(4500, 0.6); //place
            sleep(2600);
            liftHardware.moveElevator(260, 0.6);
            sleep(1000);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(3000, 1);
            liftHardware.moveElevator(0, 1);
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //keep this
            sleep(1000000);

        } else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center) {

            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 33.7, 0.0);
            robotHardware.driveStraight(0.3, -8.2, 0.0);
//            robotHardware.turnToHeading(0.3, 90);
//            robotHardware.driveStraight(0.3,15, 90);
//            robotHardware.turnToHeading(0.3, 0.0);
//            robotHardware.driveStraight(0.3, 40, 0.0);
            robotHardware.turnToHeading(0.3, -90);
            sleep(9000);

            robotHardware.driveStraight(0.5, 55, -92);
            robotHardware.turnToHeading(0.3, -89);
            robotHardware.driveStraight(0.5, 28, -89);
            robotHardware.turnToHeading(0.3, -98);
            robotHardware.driveStraight(0.1, 13.5, -96);


//            robotHardware.turnToHeading(0.3, -180.0);
//            robotHardware.driveStraight(0.3, 30.0, -180);
//            robotHardware.turnToHeading(0.3, -90.0);
//            robotHardware.driveStraight(0.1, 14.0, -94);

            liftHardware.moveElbow(4500, 0.5); //place
            sleep(2600);
            liftHardware.moveElevator(150, 0.6);
            sleep(1000);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElevator(300, 1);
            liftHardware.moveElevator(0, 1);
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //keep this
            sleep(1000000);

        } else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Left) {

            liftHardware.closeBucket();
            robotHardware.driveStraight(0.2, 20, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.2, -60);
            robotHardware.driveStraight(0.2, 10, -60);
            robotHardware.driveStraight(0.3, -10, -60);
            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 31, 0.0);
            robotHardware.turnToHeading(0.3, -92);

            robotHardware.driveStraight(0.4, 80, -93);

            robotHardware.turnToHeading(0.4, -185.0);
            robotHardware.driveStraight(0.4, 28.0, -180);
            robotHardware.turnToHeading(0.4, -96.0);
            robotHardware.driveStraight(0.1, 17.4, -98);

            liftHardware.moveElbow(4500, 0.8); //place
            sleep(2000);
            liftHardware.moveElevator(260, 0.6);
            sleep(1000);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElevator(0, 1);
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //keep this
            sleep(1000000);

        }
    }
}

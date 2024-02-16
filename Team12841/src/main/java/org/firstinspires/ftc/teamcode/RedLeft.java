package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "Red Far Side")

public class RedLeft extends LinearOpMode {
    RobotHardware robotHardware;
    LiftHardware liftHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.RedL);
        liftHardware = new LiftHardware(robotHardware, this);

        while (!opModeIsActive()) {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.addData("Blue Pos 1 Color RGB ", robotHardware.camera.pipeline.avgBlue1);
            telemetry.addData("Blue Pos 2 Color RGB ", robotHardware.camera.pipeline.avgBlue2);
            telemetry.addData("Red Pos 1 Color RGB ", robotHardware.camera.pipeline.avgRed1);
            telemetry.addData("Red Pos 2 Color RGB ", robotHardware.camera.pipeline.avgRed2);
            telemetry.update();
        }

        robotHardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        robotHardware.camera.closeCamera();

        //left position
        if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Left) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 5, 0.0);
            robotHardware.turnToHeading(0.3, -22.0);
            robotHardware.driveStraight(0.3, 17, -22.0);
            robotHardware.driveStraight(0.3, -21, -22.0);
            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 48, 0.0);

            //goes to the backboard
            robotHardware.turnToHeading(0.3, 92);
            robotHardware.driveStraight(0.3, 85, 95);
            robotHardware.turnToHeading(0.5, 187);
            robotHardware.driveStraight(0.3, 15.8, 187);
            robotHardware.turnToHeading(0.3, 97);
            robotHardware.driveStraight(0.1, 10.5, 97);

            //moves arm to place
            liftHardware.moveElbow(4480, 0.5);
            sleep(2600);
            liftHardware.moveElevator(350, 0.5);
            sleep(1000);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElevator(0, 1);
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //keep this
            sleep(1000000);

        }

        //center position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            sleep(1000);
            robotHardware.driveStraight(0.3, 33, 0.0);
            robotHardware.driveStraight(0.3, -7.5, 0);
//            robotHardware.turnToHeading(0.3, 90);
//            robotHardware.driveStraight(0.3, 13, -90);
//            robotHardware.turnToHeading(0.3, 0);
//            robotHardware.driveStraight(0.3, 31.7, 0.0);

            //goes to the backboard
            robotHardware.turnToHeading(0.3, 93);
            sleep(7000);
            robotHardware.driveStraight(0.3, 90, 95);
            robotHardware.driveStraight(0.1, 6, 95);
//            robotHardware.turnToHeading(0.3, 185.0);
//            robotHardware.driveStraight(0.3, 27.8, 190.0);
//            robotHardware.turnToHeading(0.3, 96.0);
//            robotHardware.driveStraight(0.1, 11.4, 96.0);

            //moves arm to place
            liftHardware.moveElbow(4350, 0.4); //place
            sleep(2600);
            liftHardware.moveElevator(260, 0.3);
            sleep(2000);
            liftHardware.openBucket();
            sleep(2000);
            liftHardware.moveElevator(400, 1);
            liftHardware.moveElevator(0, 1);
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //keep this
            sleep(1000000);
        }

        //right position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right) {
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 11, 0.0);
            robotHardware.turnToHeading(0.3, 32);
            robotHardware.driveStraight(0.3, 19.5, 32);
            robotHardware.turnToHeading(0.3, 40);
            robotHardware.driveStraight(0.5, -21, 40);
            robotHardware.turnToHeading(0.5, 0);
            robotHardware.driveStraight(0.3, 40, 0.0);
            robotHardware.turnToHeading(0.3, 94);
            robotHardware.driveStraight(0.3, 80, 94);

            robotHardware.turnToHeading(0.5, 184);
            robotHardware.driveStraight(0.3, 33.2, 184);
            robotHardware.turnToHeading(0.3, 96);
            robotHardware.driveStraight(0.1, 20.5, 96);

            //moves arm to place
            liftHardware.moveElbow(4450, 0.5);
            sleep(2600);
            liftHardware.moveElevator(260, 0.5);
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

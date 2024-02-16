package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "Blue Board Side")

public class BlueLeft extends LinearOpMode {
    RobotHardware robotHardware;
    LiftHardware liftHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.BlueL);
        liftHardware = new LiftHardware(robotHardware, this);
        while (opModeInInit()) {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.addData("Blue Pos 1 Color RGB ", robotHardware.camera.pipeline.avgBlue1);
            telemetry.addData("Blue Pos 2 Color RGB ", robotHardware.camera.pipeline.avgBlue2);
            telemetry.addData("Red Pos 1 Color RGB ", robotHardware.camera.pipeline.avgRed1);
            telemetry.addData("Red Pos 2 Color RGB ", robotHardware.camera.pipeline.avgRed2);
            telemetry.update();
        }

        robotHardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        robotHardware.camera.closeCamera();

        //left position
        if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Left) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 7, 0.0);
            robotHardware.turnToHeading(0.3, -19);
            robotHardware.driveStraight(0.3, 18, -22.0);
            robotHardware.driveStraight(0.3, -20, -22.0);
            robotHardware.turnToHeading(0.3, -94);
            robotHardware.driveStraight(0.3, 30, -94);
            robotHardware.turnToHeading(0.3, 0);
            robotHardware.driveStraight(0.3, 14, 0);
            robotHardware.turnToHeading(0.3, -92);
            robotHardware.driveStraight(0.2, 15, -92);

            //Moves the arm to place
            liftHardware.moveElbow(4700, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //parks
            robotHardware.driveStraight(0.3, -7, -92);
            robotHardware.turnToHeading(0.3, -180);
            robotHardware.driveStraight(0.3, 19, -180);

            //keep this
            sleep(1000000);
        }

        //middle position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 33.1, 0.0);
            robotHardware.driveStraight(0.3, -9.4, 0);
            robotHardware.turnToHeading(0.3, -90);
            robotHardware.driveStraight(0.3, 39, -90);

            //Moves the arm to place
            liftHardware.moveElbow(4640, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //parks
            robotHardware.driveStraight(0.3, -7, 0);
            robotHardware.turnToHeading(0.3, -180);
            robotHardware.driveStraight(0.3, 20, -180);

            //keep this
            sleep(1000000);
        }

        //right position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.4, 18, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.4, 52);
            robotHardware.driveStraight(0.4, 15, 52);
            robotHardware.driveStraight(0.4, -15, 52);
            robotHardware.turnToHeading(0.4, 0);
            robotHardware.driveStraight(0.3, 14, 0);
            robotHardware.turnToHeading(0.4, -93);
            robotHardware.driveStraight(0.4, 40, -93 );

            //moves the arm to place
            liftHardware.moveElbow(4650, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            //parks
            robotHardware.driveStraight(0.4, -8, -90);
            robotHardware.turnToHeading(0.4, -180);
            robotHardware.driveStraight(0.4, 30, -180);

            //keep this
            sleep(1000000000);
        }
    }
}
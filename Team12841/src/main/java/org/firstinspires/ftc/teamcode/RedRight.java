package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "Red Board Side")

public class RedRight extends LinearOpMode {
    RobotHardware robotHardware;
    LiftHardware liftHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.RedR);
        liftHardware = new LiftHardware(robotHardware, this);


        while (opModeInInit()) {
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
            robotHardware.driveStraight(0.3, 20, 0.0);
            robotHardware.turnToHeading(0.5, -60);
            robotHardware.driveStraight(0.5, 9, -60);
            robotHardware.driveStraight(0.5, -20, -60);
            robotHardware.turnToHeading(0.5, 0);
            robotHardware.driveStraight(0.5, 14.5, 0);
            robotHardware.turnToHeading(0.5, 94);
            robotHardware.driveStraight(0.3, 33.75, 94);

            //Moves the arm to place
            liftHardware.moveElbow(4500, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            robotHardware.driveStraight(0.5,-5,93);
            robotHardware.turnToHeading(0.5,180);
            robotHardware.driveStraight(0.5,35,180);


            //keep this
            sleep(1000000);
        }

        //center position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.2, 34, 0.0);
            robotHardware.driveStraight(0.3, -8.5, 0);
            robotHardware.turnToHeading(0.3, 93);
            robotHardware.driveStraight(0.3, 42, 93);

            //moves the arm to place
            liftHardware.moveElbow(4450, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);
            robotHardware.driveStraight(0.3,-5,93);
            robotHardware.turnToHeading(0.3,180);
            robotHardware.driveStraight(0.3,25,180);

            //keep this
            sleep(1000000);
        }

        //right position
        else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right) {
            //lines for placing the pixel on the spike mark
            liftHardware.closeBucket();
            robotHardware.driveStraight(0.3, 5, 0.0); //drives off the edge so it does hit the wall
            robotHardware.turnToHeading(0.3, 22.0);
            robotHardware.driveStraight(0.3, 21.5, 22.0);
            robotHardware.driveStraight(0.3, -8 , 22.0);
            robotHardware.turnToHeading(0.3, 94);
            robotHardware.driveStraight(0.3, 37, 94);

            //moves the arm to place
            liftHardware.moveElbow(4450, 1);
            sleep(2500);
            liftHardware.openBucket();
            sleep(1000);
            liftHardware.moveElbow(0, 1);

            robotHardware.driveStraight(0.3, -5.5, 94);
            robotHardware.turnToHeading(0.3, 179);
            robotHardware.driveStraight(0.3, 20.0, 179);

            //keep this
            sleep(1000000);
        }
    }
}
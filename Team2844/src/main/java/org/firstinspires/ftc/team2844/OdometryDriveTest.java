package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.OdometryDrive;

@Autonomous(name="OdometryTest")
public class OdometryDriveTest extends LinearOpMode {

   // OdometryGlobalCoordinatePosition globalPositionUpdate;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        OdometryDrive odometryDrive = new OdometryDrive(robot);

        waitForStart();
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
       // globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.leftVerticalEncoder, robot.rightVerticalEncoder, robot.horizontalEncoder, robot.OD_COUNTS_PER_INCH, 75);
       // Thread positionThread = new Thread(globalPositionUpdate);
       // positionThread.start();

        telemetry.addData("Thread Active", odometryDrive.positionThread.isAlive());
        telemetry.update();
       // while (opModeIsActive());

        odometryDrive.goToPositionForward(0,10,.5,0,0.5);
         while (opModeIsActive());
     //   globalPositionUpdate.stop();
    }
}

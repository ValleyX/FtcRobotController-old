package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.OdometryDrive;
@Disabled
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

        odometryDrive.goToPositionForward(0,51.5,.5,0,0.05);
       //  while (opModeIsActive());
     //   globalPositionUpdate.stop();

        //power .5
        // ValleyX: robotMovementY 0.0048253325468411365 FAILED
        // ValleyX: robotMovementY -0.003846342625651588 SUCCESS
        //ValleyX: robotMovementY 0.01238885224315174 SUCCESS
        //ValleyX: robotMovementY -0.01019884227190645 SUCCCESS
        //ValleyX: robotMovementY 0.006914251608113086 SUCCESS

        //power 1
        //ValleyX: robotMovementY -8.57637927709664E-4 janky succes
        //power.08
        // ValleyX: robotMovementY 0.006415866465911877 FAILED
        //.6 power
        // ValleyX: robotMovementY -0.008403732479550377 Success but took longer than .5 power
    }
}

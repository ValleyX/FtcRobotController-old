package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.OdometryDrive;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Disabled
@Autonomous(name="OdStrafeTest")
public class OdStrafeTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        EncoderDriveMecha mechaDrive = new EncoderDriveMecha(robot);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        OdometryDrive odometryDrive = new OdometryDrive(robot); // recently added to test
        LiftMaths liftMaths = new LiftMaths(robot);
        // final double elbowSpeed = 0.6;
        final double elbowUpSpeed = 0.3;
        final double elbowDownSpeed = 0.2;
        final int repetitions = 6;
        final double turnTableSpeed = 0.5;
        RobotHardware.PowerPlayPipeline.MarkerPosition alphaColor = RobotHardware.PowerPlayPipeline.MarkerPosition.Green;

        RobotArmDriver_Position armDriverPos = new RobotArmDriver_Position(robot);
        // armDriverPos.elbowToPosition(elbowUpSpeed,1,true);

        System.out.println("valleyX: elbow start " + robot.elbow.getCurrentPosition());

        robot.elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeInInit()) {
            //telemetry.addData("red value", robotHardware_.pipeline.avgLeftRed);
            //telemetry.addData("blue value", robotHardware_.pipeline.avgLeftBlue);

            telemetry.addData("r value", robot.pipeline.avgLeftR);
            telemetry.addData("g value", robot.pipeline.avgLeftG);
            telemetry.addData("b value", robot.pipeline.avgLeftB);

            telemetry.addData("alpha color", robot.pipeline.color);
            telemetry.addData("Thread Active", odometryDrive.positionThread.isAlive());
            telemetry.update();
            alphaColor = robot.pipeline.color;
            idle();

        }
        ElbowPID2 elbowPIDThread = new ElbowPID2(robot);
        Thread elbowThread = new Thread(elbowPIDThread);
        elbowThread.start();

        armDriverPos.setClawPos(0, true);
        sleep(150);
        armDriverPos.winchToPosition(1,4,true);
        elbowPIDThread.runElbow(84,13);
        sleep(8000);
        odometryDrive.goToPositionSide(21.5,0,1,0,1);
         sleep(10000);
        odometryDrive.goToPositionSide(0,0,1,0,1);

        /*
        if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green) {

            //gyroMove.driveStraight(1,5,0);

        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Red){
            //gyroMove.turnToHeading(.5, -90);
            //gyroMove.driveStraight(.5, 24.5, 270);

            //gyroMove.driveStraight(1, 24,0);
            gyroMove.turnToHeading(0.5,-90);
            odometryDrive.goToPositionForward(0,26,.5,0,0.05);
           // gyroMove.driveStraight(1,30 ,-90);
        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Blue){
            gyroMove.turnToHeading(.25, 90);
            odometryDrive.goToPositionForward(0,26,.5,0,0.05);
            gyroMove.driveStraight(.25, 26, -270);
        }
*/





        robot.allpower(0);


    }



}


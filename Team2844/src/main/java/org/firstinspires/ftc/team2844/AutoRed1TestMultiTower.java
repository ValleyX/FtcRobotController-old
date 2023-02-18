package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.OdometryDrive;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Disabled
@Autonomous(name="LeftAutoMultiTower")
public class AutoRed1TestMultiTower extends LinearOpMode {



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
        int repetitions = 3;//6
        final double turnTableSpeed = 0.5;
        RobotHardware.PowerPlayPipeline.MarkerPosition alphaColor = RobotHardware.PowerPlayPipeline.MarkerPosition.Green;

        RobotArmDriver_Position armDriverPos = new RobotArmDriver_Position(robot);
        // armDriverPos.elbowToPosition(elbowUpSpeed,1,true);

        System.out.println("valleyX: elbow start " + robot.elbow.getCurrentPosition());

        while (opModeInInit()) {

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

        //Putting cone onto nearest tower
        elbowPIDThread.runElbow(8,10);
        armDriverPos.setClawPos(robot.clawClose, true);
        elbowPIDThread.runElbow(20,10);




        //Putting cone onto Tallest tower


        //Putting tower onto middle tower




        if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green) {

            //gyroMove.driveStraight(1,5,0);
            elbowPIDThread.runElbow(8,20);
            armDriverPos.winchToPosition(1,0,false);
            armDriverPos.turnTableToPosition(0.8,0,false);
            odometryDrive.goToPositionForward(0,50,0.8,0,1);


        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Red){
            elbowPIDThread.runElbow(84,20);
            armDriverPos.winchToPosition(1,0,false);
            armDriverPos.turnTableToPosition(0.8,0,false);
            odometryDrive.goToPositionSide(-16.5,51.5,0.5,0,1);
            odometryDrive.goToPositionForward(-16.5,50,0.5,0,1);


        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Blue){
            armDriverPos.setClawPos(0, true);
            sleep(150);
            armDriverPos.winchToPosition(1,4,true);
            elbowPIDThread.runElbow(84,13);
            odometryDrive.goToPositionSide(26,51.5,0.8,0,1);
            odometryDrive.goToPositionForward(26,50,0.8,0,1);

            armDriverPos.turnTableToPosition(0.8,0,true);
        }






        //robot.allpower(0);


    }



}


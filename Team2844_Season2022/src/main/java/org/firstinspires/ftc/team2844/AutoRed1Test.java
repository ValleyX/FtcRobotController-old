package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.OdometryDrive;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Disabled
@Autonomous(name="LeftAutoTest")
public class AutoRed1Test extends LinearOpMode {



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
        ElapsedTime timer = new ElapsedTime();
        int repetitions = 3;//6
        final double turnTableSpeed = 0.5;
        RobotHardware.PowerPlayPipeline.MarkerPosition alphaColor = RobotHardware.PowerPlayPipeline.MarkerPosition.Green;

        RobotArmDriver_Position armDriverPos = new RobotArmDriver_Position(robot);
        // armDriverPos.elbowToPosition(elbowUpSpeed,1,true);

       // System.out.println("valleyX: elbow start " + robot.elbow.getCurrentPosition());

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

            if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green)
            {
                repetitions = 5;
            }
            else
            {
                repetitions = 4;
            }

        }
        //System.out.println("valleyX: elbow before pid " + robot.elbow.getCurrentPosition());

       // elbowPID elbowPIDThread = new elbowPID(robot);

        ElbowPID2 elbowPIDThread = new ElbowPID2(robot);
        Thread elbowThread = new Thread(elbowPIDThread);
        elbowThread.start();

        //System.out.println("valleyX: elbow after pid " + robot.elbow.getCurrentPosition());
        //System.out.println("valleyX: elbow after pid " + robot.elbow2.getCurrentPosition());

        final int clawDelay = 150;
        //final int sleepOut = 810;
        final int sleepOut = 1010; //1010
        //slow auto test with sleeps
        elbowPIDThread.runElbow(8,3);
        System.out.println("valleyX: clawclose final target ticks ");
        armDriverPos.setClawPos(robot.clawClose, true);
        sleep(clawDelay);
       // elbowPIDThread.runElbow(5,2);
      //  odometryDrive.goToPositionForward(0,51.5,.5,0,0.05); //ADD BACK IN AT THE END!!!!!!!!!!!! this is removed for more efficient testing

        final double turntablespeed = 0.7;
        final int elbowPos = 84;//was 84  //78
        final double winchHeight = 48.56; //was 49.2, was 47.56    44.1
        final double initialAngleToPol = -27.5;
        final double angleToPol = -37.5; //was 39.0
        armDriverPos.winchToPosition(1,10,true);
        elbowPIDThread.runElbow(elbowPos,13); //4 is pretty consistant
        armDriverPos.turnTableToPosition(turntablespeed, initialAngleToPol, false); // turning to tower
      //  odometryDrive.goToPositionForward(0,10,0.5,0,5);
      //  odometryDrive.goToPositionForward(0,20,.75,0,5);
        odometryDrive.goToPositionForward(0,45.5,.7,0,4); //ADD BACK IN AT THE END!!!!!!!!!!!! this is removed for more efficient testing
        odometryDrive.goToPositionForward(0,51.5,0.5,0,1); //ADD BACK IN AT THE END!!!!!!!!!!!! this is removed for more efficient testing

        armDriverPos.turnTableToPosition(turntablespeed, angleToPol, false); // turning to tower
        //sleep(3000);
        //elbowPIDThread.runElbow(84,13); //4 is pretty consistant

      //  int[] stackAngle = {16,14,12, 5, 1};
        int[] stackAngle = {16,15,13, 11, 1};

        System.out.println("valleyX: reetitions " + repetitions);
        for (int i = 0; (i < repetitions) && opModeIsActive(); i++)
        {

            int target =0;
            int current =0;
            boolean Executed = false;
  //          while ((/*robot.turnTable.isBusy()*/ (!armDriverPos.turnTableCloseEnough() || elbowPIDThread.isBusy())) && opModeIsActive() /*|| (Math.abs((target = robot.elbow.getTargetPosition()) - (current = robot.elbow.getCurrentPosition())) > 2) */ ) {
            timer.reset();
            while ((/*robot.turnTable.isBusy()*/ (!armDriverPos.turnTableCloseEnough() || elbowPIDThread.isBusy())) && opModeIsActive() && timer.milliseconds() <= sleepOut /*|| (Math.abs((target = robot.elbow.getTargetPosition()) - (current = robot.elbow.getCurrentPosition())) > 2) */ ) {
                   /*
                System.out.println("\nvalleyX: elbow2 ticks " + -robot.elbow2.getCurrentPosition());
                System.out.println("valleyX: elbow2 target ticks " + elbowPIDThread.targetTicks());
                System.out.println("valleyX: elbow2 elbow is busy "+ elbowPIDThread.isBusy() + "\n");
                System.out.println("valleyX: elbow2 winch is busy "+ robot.winch.isBusy() + "\n");

                    */
                if (!robot.turnTable.isBusy() && !Executed)
                {
                    armDriverPos.winchToPosition(1, winchHeight, true);
                    Executed = true;
                }

                idle();
            }
            /*
            System.out.println("valleyX: elbow2 elbow is busy "+ elbowPIDThread.isBusy() + "\n");
            System.out.println("\nvalleyX: elbow2 final ticks " + -robot.elbow2.getCurrentPosition());
            System.out.println("valleyX: elbow2 final target ticks " + elbowPIDThread.targetTicks());
            */
            armDriverPos.winchToPosition(1, winchHeight, false);

            timer.reset();
            while (robot.winch.isBusy()  && opModeIsActive() && (timer.milliseconds() <= 1600)) {
                idle();
            }

           // sleep(100);//needed was 150

            //armDriverPos.winchToPosition(1, 45, true);

            System.out.println("valleyX: clawopen final target ticks ");
            armDriverPos.setClawPos(robot.clawOpen, true);

            //sleep(360);
            sleep(clawDelay);
            if (i==repetitions-1) {
                System.out.println("exiting loop");
                break;
            }


            armDriverPos.winchToPosition(1, 10, false);
            armDriverPos.turnTableToPosition(turntablespeed, 75, false);
            timer.reset();
            while ((robot.winch.isBusy() || /*robot.turnTable.isBusy()*/ !armDriverPos.turnTableCloseEnough() ) && opModeIsActive() && (timer.milliseconds() <= sleepOut )) {
                idle();
            }

            armDriverPos.turnTableToPosition(.5, 85, false);

            //go to cone

            //System.out.println("valleyX: elbow before second rise " + robot.elbow.getCurrentPosition());
           // list of auto cone elbow anglr, 22 cone 1, 20 cone 2, 16 , 14.5 , 11
            elbowPIDThread.runElbow(stackAngle[i], 0); //15
            armDriverPos.winchToPosition(1, 33 + 0.1*i, false);


            timer.reset();
            while ((robot.winch.isBusy() || elbowPIDThread.isBusy()) && (opModeIsActive() && timer.milliseconds() <= 1500)) {
                idle();//idle
            }

            armDriverPos.winchToPosition(1, 39, true);
           // sleep(200);

            System.out.println("valleyX: clawclose final target ticks ");
            armDriverPos.setClawPos(robot.clawClose, true);
            sleep(clawDelay);
            armDriverPos.winchToPosition(1, 38, false);
            elbowPIDThread.runElbow(31,0);//150
            timer.reset();
            while ((robot.winch.isBusy() || elbowPIDThread.isBusy()) && opModeIsActive() && timer.milliseconds() <= sleepOut) {
                idle();
            }
            //armDriverPos.elbowToPosition(1, 31, true);
            armDriverPos.winchToPosition(1, 10, false);
            armDriverPos.turnTableToPosition(turntablespeed, angleToPol, false); //turning to tower
            elbowPIDThread.runElbow(elbowPos,0); //was 13
            timer.reset();
			/*
            while ((robot.winch.isBusy() || elbowPIDThread.isBusy()) && opModeIsActive() && timer.milliseconds() <= sleepOut) {
                idle();
            }
            timer.reset();
			*/
            while ((robot.winch.isBusy() || robot.turnTable.isBusy() && !armDriverPos.turnTableCloseEnough())  && opModeIsActive()  && timer.milliseconds() <= sleepOut ){
                idle();
            }


        }

        System.out.println("valleyX: clawopen final target ticks ");
        armDriverPos.setClawPos(robot.clawOpen, true);
        sleep(clawDelay);



        if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green) {

            elbowPIDThread.runElbow(80,10);
            armDriverPos.winchToPosition(1,5,false);
            armDriverPos.turnTableToPosition(0.8,0,false);
            odometryDrive.goToPositionForward(0,50,0.8,0,1);

            timer.reset();
            while ((robot.winch.isBusy() || /*robot.turnTable.isBusy()*/ !armDriverPos.turnTableCloseEnough())  && opModeIsActive()){
                idle();
            }
            //odometryDrive.goToPositionForward(0,50,0.8,0,1);


        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Red){
            elbowPIDThread.runElbow(84,20);
            armDriverPos.winchToPosition(1,0,false);
            armDriverPos.turnTableToPosition(0.8,0,false);
     //       while ((robot.winch.isBusy() || /*robot.turnTable.isBusy()*/ !armDriverPos.turnTableCloseEnough())  && opModeIsActive()){
     //           idle();
      //      }
            odometryDrive.goToPositionSide(-18,51.5,0.5,0,1);
            odometryDrive.goToPositionForward(-16.5,50,0.5,0,1);


        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Blue){

            elbowPIDThread.runElbow(84,13);
            armDriverPos.winchToPosition(1,4,false);
            armDriverPos.turnTableToPosition(0.8,0,false);
            //while ((robot.winch.isBusy() || /*robot.turnTable.isBusy()*/ !armDriverPos.turnTableCloseEnough())  && opModeIsActive()){
            //    idle();
            //}
            odometryDrive.goToPositionSide(26,51.5,0.8,0,1);
            odometryDrive.goToPositionForward(26,50,0.8,0,1);

            armDriverPos.turnTableToPosition(0.8,0,true);
        }
        System.out.println("valleyX: clawclose final target ticks ");
        armDriverPos.setClawPos(robot.clawClose, true);
        sleep(clawDelay);
        elbowPIDThread.runElbow(5,2);
        while (elbowPIDThread.isBusy() && opModeIsActive())
        {
            idle();
        }


    }



}


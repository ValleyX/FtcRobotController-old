package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;


@Autonomous(name="RightAuto")
public class AutoRed2 extends LinearOpMode {

    double wristPos = 0;
    double elbowpos = 0;
    final double elbowUpSpeed = 0.3;
    final double elbowDownSpeed = 0.1;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        EncoderDriveMecha mechaDrive = new EncoderDriveMecha(robot);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);

        RobotHardware.PowerPlayPipeline.MarkerPosition alphaColor = RobotHardware.PowerPlayPipeline.MarkerPosition.Green;

        RobotArmDriver_Position armDriverPos = new RobotArmDriver_Position(robot);

        while(opModeInInit()){
            //telemetry.addData("red value", robotHardware_.pipeline.avgLeftRed);
            //telemetry.addData("blue value", robotHardware_.pipeline.avgLeftBlue);

            telemetry.addData("r value", robot.pipeline.avgLeftR);
            telemetry.addData("g value", robot.pipeline.avgLeftG);
            telemetry.addData("b value", robot.pipeline.avgLeftB);

            telemetry.addData("alpha color", robot.pipeline.color);

            telemetry.update();
            alphaColor = robot.pipeline.color;
        }

       // armDriverPos.setClawPos(1, true);

        waitForStart();

        //IMU testing stuff, ignore it unless you want to begin your suffer journey
        //gyroMove.turnToHeading(0.7, -90);

        //sleep(5000);

        //gyroMove.driveStraight(.5, -120, 0);

        //this is test code to make sure the robot is going straight, if you delete this i will strangle you

       /// gyroMove.driveStraight(.5, -50, 0);
      //  gyroMove.turnToHeading(0.5, 0);


        //grab cone at start of match
        armDriverPos.setClawPos(robot.clawClose, true);
        sleep(500);

        //extend a tad bit so it doesnt clip the polycartb
        armDriverPos.elbowToPosition(elbowUpSpeed,1,true);
        armDriverPos.winchToPosition(1, 2, true);

        //move to 1st position
        gyroMove.driveStraight(.5, -50, 0);
        //gyroMove.turnToHeading(0.3, 0);
        //gyroMove.driveStraight(.5, 6, 0);
        //gyroMove.turnToHeading(0.3, 0);

        //Raise lift at beginning
        armDriverPos.elbowToPosition(-.5, 74,true);
        sleep(250);

       // sleep(500);

        //extend arm to score height//boat
        armDriverPos.winchToPosition(1, 44, true);
        sleep(250);

        //turn turntable to face pole
        armDriverPos.turnTableToPosition(.5, 37, true);
        sleep(250);

        //drop cone first time
        armDriverPos.setClawPos(robot.clawOpen, true);
        sleep(250);

        //retract lift
        armDriverPos.winchToPosition(1, 10, true);

        //turn to pick up cone
        armDriverPos.turnTableToPosition(.5, -83, true);

        //lower arm elbow
        armDriverPos.elbowToPosition(-.5, 21, true);

        //extend arm
        armDriverPos.winchToPosition(1, 37, true);

        //grab the cone
        armDriverPos.setClawPos(robot.clawClose, true);
        sleep(250);

        //pull back arm slightly as to miss the rim of the arena
        armDriverPos.winchToPosition(1, 32, true);
        sleep(250);

        //move elbow up again to remove cone
        armDriverPos.elbowToPosition(-.2, 75, true);
        sleep(100);

        //extend the arm to scoring height
        armDriverPos.winchToPosition(1, 46, true);
        sleep(100);

        //move turntable right to large tower
        armDriverPos.turnTableToPosition(.5, 38, true);
        sleep(800);

        //open claw to drop cone
        armDriverPos.setClawPos(robot.clawOpen, true);
        sleep(500);

        //makes the robot not commit soduko when it finishes the running
        armDriverPos.turnTableToPosition(.5, 0, true);
        armDriverPos.winchToPosition(1, 6, true);
        //armDriverPos.elbowToPosition(-.5, 20, true);
        //armDriverPos.elbowToPosition(-.5, 5, true);

        //lower arm elbow
        armDriverPos.elbowToPosition(-.5, 5, true);
        sleep(500);
        armDriverPos.setClawPos(robot.clawOpen, true);


        //this is the movement code that does stuff off of camera detection, commmented out for test, you'll need this
         if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green) {
            gyroMove.driveStraight(0.5,5,0);

        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Red){
            gyroMove.turnToHeading(.5, 90);
            gyroMove.driveStraight(.5, -24.5, 90);
        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Blue){
            gyroMove.turnToHeading(.25, -90);
            gyroMove.driveStraight(.25, -25, -90);
        }



        //robot.allpower(0);

    }


}

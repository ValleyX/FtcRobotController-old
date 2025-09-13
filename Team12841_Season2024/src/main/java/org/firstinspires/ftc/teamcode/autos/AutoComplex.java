package org.firstinspires.ftc.teamcode.autos;
//Muwah ha ha ha ha
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "Complex Auto(Basket Side)")
@Disabled
public class AutoComplex extends LinearOpMode {
    // Why is Carter so "great" at coding?
    RobotHardware robotHardware;
    LiftHardware liftHardware;


    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        liftHardware = new LiftHardware(robotHardware, this);
        waitForStart();

        robotHardware.imu.resetYaw();
        liftHardware.closeClaw();
        sleep(1000);
        robotHardware.driveStraight(0.3,25,0); //move toward the submersible
        liftHardware.moveYSlidesAuto(26.9, 1); //moves the ySlides up above the second bar
        robotHardware.driveStraight(0.3,12,0); //move toward the submersible
        sleep(200);
        liftHardware.moveYSlidesAuto(24.5, 1); //moves ySlides down to the bar
        robotHardware.driveStraight(0.3, -7, 0);
        liftHardware.openClaw(); //let go of speciman
        sleep(200); //wait for claw to open


        robotHardware.driveStraight(0.3,-5,0); //move slightly away from the submersible
        liftHardware.closeClaw();
        liftHardware.moveYSlidesAuto(0, 1); //put the slides down
        robotHardware.turnToHeading(0.3,90); //rotate towards the left at 90 degrees
        robotHardware.driveStraight(0.3,30,90); //move towards the left, and drive 30in
        robotHardware.turnToHeading(0.3, 0); // rotate facing the front
        robotHardware.driveStraight(0.3, 26, 0); // drive forward near the enemies zone without crossing the line
        robotHardware.turnToHeading(0.5, 90);
        robotHardware.driveStraight(0.3, 10, 90);
        robotHardware.turnToHeading(0.3, -5);
        robotHardware.driveStraight(0.6, -53, -5); // move towards the net zone
        sleep(200);
        robotHardware.driveStraight(0.5, 15, -5);
        robotHardware.turnToHeading(0.3, 0);
    }
}
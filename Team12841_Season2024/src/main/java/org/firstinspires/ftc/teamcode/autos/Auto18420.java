package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@Autonomous(name = "Auto 18420 (Kind)")
@Disabled
public class Auto18420 extends LinearOpMode {
    // Why is Carter so "great" at coding?
    RobotHardware robotHardware;
    LiftHardware liftHardware;


    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        liftHardware = new LiftHardware(robotHardware, this);
        waitForStart();

        robotHardware.imu.resetYaw();
        liftHardware.closeClaw();
        liftHardware.moveYSlides(4, 1);
        sleep(10000);
        robotHardware.driveStraight(0.3, 5, 0);
        robotHardware.turnToHeading(0.5, -90);
        robotHardware.driveStraight(0.5, 33, -90);
        robotHardware.turnToHeading(0.3, 0);

        liftHardware.moveYSlidesAuto(27, 1); //moves the ySlides up above the second bar
        sleep(200);
        robotHardware.driveStraight(0.3,28,0); //move toward the submersible
        sleep(200);
        liftHardware.moveYSlidesAuto(24.5, 1); //moves ySlides down to the bar
        robotHardware.driveStraight(0.3, -7, 0);
        liftHardware.openClaw(); //let go of speciman
        sleep(200); //wait for claw to open


        robotHardware.driveStraight(0.3,-5,0); //move slightly away from the submersible
        liftHardware.moveYSlides(0, 1); //put the slides down
        liftHardware.closeClaw();
        robotHardware.turnToHeading(90, 0.5);
        robotHardware.driveStraight(0.5, 33, 90);
    }
}
package org.firstinspires.ftc.team12841;


import static org.firstinspires.ftc.team12841.RobotGyroscope.DRIVE_SPEED;
import static org.firstinspires.ftc.team12841.RobotGyroscope.TURN_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.drivers.EncoderDrive_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.LiftDriver_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.RobotHardware;

@Autonomous(name = "RookieAutonomousRightSide")
public class RookieAtonomous extends LinearOpMode {

    RobotHardware robotHardware;
    EncoderDrive_UnderthehoodStuff encoderDrive;
    LiftDriver_UnderthehoodStuff liftdriver;


    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        RobotGyroscope Gyro = new RobotGyroscope(robotHardware);
        liftdriver = new LiftDriver_UnderthehoodStuff(robotHardware);

        while (opModeInInit())
        {
            telemetry.addData("coneColor", robotHardware.pipeline.markerPos);
            telemetry.addData("gyro", robotHardware.imu.isGyroCalibrated());
            telemetry.update();

        }

        if (robotHardware.pipeline.markerPos == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.Red)
        {
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            Gyro.driveStraight(.3,3,0);
            Gyro.driveStraight(.3,29,0);
            liftdriver.moveInches(22,1);
            Gyro.turnToHeading(.3,-40, 10000000);
            sleep(1000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(.3,0, 10000000);
            Gyro.driveStraight(0.3,30,0);//dowwnload bees
            Gyro.driveStraight(0.3,-7,0);
            Gyro.turnToHeading(0.3,90, 1000000);
            liftdriver.moveInches(5,.5);
            Gyro.driveStraight(0.3,21,90);
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(500);
            liftdriver.moveInches(22,0.5);
            Gyro.driveStraight(0.3,-21,90);
            Gyro.turnToHeading(0.3,-140, 10000000);
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(0.1,-90, 1000000);
            liftdriver.moveInches(0,0.3);
            Gyro.driveStraight(0.2,23,-90);
            Gyro.turnToHeading(0.2,0, 1000000);
            sleep(1000);
        }
        else if (robotHardware.pipeline.markerPos == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.Green)
        {
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            Gyro.driveStraight(.3,3,0);
            Gyro.driveStraight(.3,29,0);
            liftdriver.moveInches(22,1);
            Gyro.turnToHeading(.3,-45, 1000000);
            sleep(1000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(.3,0, 1000000);
            Gyro.driveStraight(0.3,30,0);//dowwnload bees
            Gyro.driveStraight(0.3,-6,0);
            Gyro.turnToHeading(0.3,90, 1000000);
            liftdriver.moveInches(5,.5);
            Gyro.driveStraight(0.3,21,90);
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(500);
            liftdriver.moveInches(22,0.5);
            Gyro.driveStraight(0.3,-21,90);
            Gyro.turnToHeading(0.3,-140, 1000000);
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(0.3,90, 1000000);
            liftdriver.moveInches(0,0.3);
            sleep(1000);
        }
        else
        {
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            Gyro.driveStraight(.3,3,0);
            Gyro.driveStraight(.3,29,0);
            liftdriver.moveInches(22,1);
            Gyro.turnToHeading(.3,-40, 1000000);
            sleep(1000);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(.3,0, 1000000);
            Gyro.driveStraight(0.3,30,0);//dowwnload bees
            Gyro.driveStraight(0.3,-7,0);
            Gyro.turnToHeading(0.3,90, 1000000);
            liftdriver.moveInches(5,.5);
            Gyro.driveStraight(0.3,21,90);
            robotHardware.pinch.setPosition(robotHardware.closedPinch);
            sleep(500);
            liftdriver.moveInches(22,0.5);
            Gyro.driveStraight(0.3,-21,90);
            Gyro.turnToHeading(0.3,-140, 1000000);
            sleep(500);
            robotHardware.pinch.setPosition(robotHardware.openPinch);
            Gyro.turnToHeading(0.1,-90, 1000000);
            liftdriver.moveInches(0,0.3);
            Gyro.driveStraight(0.3,-24,-90);
            //Gyro.turnToHeading(0.3,0, 1000000);
            sleep(1000);
        }
    }
}
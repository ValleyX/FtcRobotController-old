package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import java.nio.file.Watchable;

@Autonomous(name = "red wheel")

public class RedWheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this, RobotHardware.CamX1,RobotHardware.CamY, RobotHardware.CamX2, RobotHardware.CamY, RobotHardware.CamX3, RobotHardware.CamY,  RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));
        LiftDrive liftDrive = new LiftDrive(robotHardware);
        //RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPosFound;
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPosFound = RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER;


        //waitForStart();
        while (!isStarted())
        {
            telemetry.addData("Team Marker Pos", robotHardware.pipeline.markerPos);
            telemetry.update();
            markerPosFound = robotHardware.pipeline.markerPos;
        }
//this all needs reversing:

        if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.LEFT){
            liftDrive.StartAction(.5, 4, 5, true);
            encoderDrive.StartAction(0.5, -5.5, -5.5, 5, true);
            encoderDrive.StartAction(0.5, -8.5, 8.5, 5, true);
            encoderDrive.StartAction(0.5, -18.5, -18.5, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 8, 8, 5, true);
            encoderDrive.StartAction(0.5, -11, 11, 5, true);
            encoderDrive.StartAction(0.5, 29.5, 29.5, 5, true);
            encoderDrive.StartAction(0.5, 18.5, -18.5, 5, true);
            encoderDrive.StartAction(1, 11, 11, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.03);
            sleep(8500);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);



        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER){
            liftDrive.StartAction(.5, 10, 5, true);
            encoderDrive.StartAction(0.5, -5.5, -5.5, 5, true);
            encoderDrive.StartAction(0.5, -8.5, 8.5, 5, true);
            encoderDrive.StartAction(0.5, -18.5, -18.5, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 8, 8, 5, true);
            encoderDrive.StartAction(0.5, -11, 11, 5, true);
            encoderDrive.StartAction(0.5, 30, 30, 5, true);
            encoderDrive.StartAction(0.5, 18.5, -18.5, 5, true);
            encoderDrive.StartAction(0.5, 12.5, 12.5, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.01);
            sleep(4000);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);


        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.RIGHT){
            liftDrive.StartAction(.5, 13, 5, true);
            encoderDrive.StartAction(0.5, -5.5, -5.5, 5, true);
            encoderDrive.StartAction(0.5, -8.5, 8.5, 5, true);
            encoderDrive.StartAction(0.4, -18.225, -18.225, 5, true);
            robotHardware.InServo.setPosition(.65);
            sleep(600);
            robotHardware.InMotor.setPower(.7);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            robotHardware.InServo.setPosition(.8);
            encoderDrive.StartAction(0.5, 8, 8, 5, true);
            encoderDrive.StartAction(0.5, -11, 11, 5, true);
            encoderDrive.StartAction(0.5, 30, 30, 5, true);
            encoderDrive.StartAction(0.5, 18.5, -18.5, 5, true);
            encoderDrive.StartAction(0.3, 11, 11, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.01);
            sleep(6000);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);




        }
}}



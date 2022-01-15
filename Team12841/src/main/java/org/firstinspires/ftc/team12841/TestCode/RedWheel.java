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
            encoderDrive.StartAction(0.5, 29, 29, 5, true);
            encoderDrive.StartAction(0.5, 17, -17, 5, true);
            encoderDrive.StartAction(.1, 11, 11, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.01);
            sleep(6500);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, 4, -4, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);
            encoderDrive.StartAction(0.5, -2.5, 2.5, 5, true);



        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER){
            liftDrive.StartAction(.5, 10, 5, true);
            encoderDrive.StartAction(0.5, -5.5, -5.5, 5, true);
            encoderDrive.StartAction(0.5, -8.5, 8.5, 5, true);
            encoderDrive.StartAction(0.5, -18.5, -18.5, 5, true);
            robotHardware.InMotor.setPower(.65);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 8, 8, 5, true);
            encoderDrive.StartAction(0.5, -11, 11, 5, true);
            encoderDrive.StartAction(0.5, 29, 29, 5, true);
            encoderDrive.StartAction(0.5, 17,-17, 5, true);
            encoderDrive.StartAction(0.1, 10.98, 10.98, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.01);
            sleep(4500);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, 3, -3, 5, true);
            encoderDrive.StartAction(0.5, -16, -16, 5, true);


        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.RIGHT){
            liftDrive.StartAction(.5, 13, 5, true);
            encoderDrive.StartAction(0.5, -35.5, -35.5, 5, true);
            encoderDrive.StartAction(0.5, 30, 30, 5, true);
            encoderDrive.StartAction(0.5, -8.5, 8.5, 5, true);
            encoderDrive.StartAction(0.4, -18.75, -18.75, 5, true);
            robotHardware.InServo.setPosition(.65);
            sleep(600);
            robotHardware.InMotor.setPower(.9);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            robotHardware.InServo.setPosition(.8);
            encoderDrive.StartAction(0.5, 8.525, 8.525, 5, true);
            encoderDrive.StartAction(0.5, -11, 11, 5, true);
            encoderDrive.StartAction(0.5, 29, 29, 5, true);
            encoderDrive.StartAction(0.5, 17, -17, 5, true);
            encoderDrive.StartAction(0.1, 9.3, 9.3, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(-1);
            robotHardware.allpower(0.015);
            sleep(4500);
            robotHardware.WheelMotor.setPower(0);
            robotHardware.allpower(0);
            encoderDrive.StartAction(0.5, 4.5,-4.5, 5, true);
            encoderDrive.StartAction(0.5, -16, -16, 5, true);





        }
}}



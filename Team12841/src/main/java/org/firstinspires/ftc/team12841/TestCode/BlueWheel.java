package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import java.nio.file.Watchable;

@Autonomous(name = "blu wheel")

public class BlueWheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this, RobotHardware.CamX1,RobotHardware.CamY, RobotHardware.CamX2, RobotHardware.CamY, RobotHardware.CamX3, RobotHardware.CamY, RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));
        LiftDrive liftDrive = new LiftDrive((robotHardware));
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPos;
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPosFound = RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER;

        //waitForStart();
        while (!isStarted())
        {
            telemetry.addData("Team Marker Pos", robotHardware.pipeline.markerPos);
            telemetry.update();
            markerPosFound = robotHardware.pipeline.markerPos;
        }


        if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.LEFT){
            liftDrive.StartAction(.5, 4, 5, true);
            encoderDrive.StartAction(0.5, -27, -27, 5, true);
            encoderDrive.StartAction(0.5, 15, -15, 5, true);
            encoderDrive.StartAction(0.5, -9, -9, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 4, 4, 5, true);
            encoderDrive.StartAction(0.5, -6, 6, 5, true);
            encoderDrive.StartAction(0.5, 32, 32, 5, true);
            encoderDrive.StartAction(0.5, 8, -8, 5, true);
            encoderDrive.StartAction(0.5, 2, 2, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(1);
            sleep(4000);
            robotHardware.WheelMotor.setPower(0);
            encoderDrive.StartAction(0.5, -18.5, 18.5, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);


        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER){
            liftDrive.StartAction(.5, 10, 5, true);
            encoderDrive.StartAction(0.5, -27, -27, 5, true);
            encoderDrive.StartAction(0.5, 15, -15, 5, true);
            encoderDrive.StartAction(0.5, -9, -9, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 4, 4, 5, true);
            encoderDrive.StartAction(0.5, -6, 6, 5, true);
            encoderDrive.StartAction(0.5, 32, 32, 5, true);
            encoderDrive.StartAction(0.5, 8, -8, 5, true);
            encoderDrive.StartAction(0.5, 2, 2, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(1);
            sleep(4000);
            robotHardware.WheelMotor.setPower(0);
            encoderDrive.StartAction(0.5, -18.5, 18.5, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);


        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.RIGHT){
            liftDrive.StartAction(.5, 13, 5, true);
            encoderDrive.StartAction(0.5, -27, -27, 5, true);
            encoderDrive.StartAction(0.5, 15, -15, 5, true);
            encoderDrive.StartAction(0.5, -9, -9, 5, true);
            robotHardware.InServo.setPosition(.65);
            sleep(600);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            robotHardware.InServo.setPosition(.8);
            encoderDrive.StartAction(0.5, 4, 4, 5, true);
            encoderDrive.StartAction(0.5, -6, 6, 5, true);
            encoderDrive.StartAction(0.5, 32, 32, 5, true);
            encoderDrive.StartAction(0.5, 8, -8, 5, true);
            encoderDrive.StartAction(0.5, 2, 2, 5, true);
            //Below is the turntable spinning motor
            robotHardware.WheelMotor.setPower(1);
            sleep(4000);
            robotHardware.WheelMotor.setPower(0);
            encoderDrive.StartAction(0.5, -18.5, 18.5, 5, true);
            encoderDrive.StartAction(0.5, -17, -17, 5, true);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);
            encoderDrive.StartAction(0.5, -1, -1, 5, true);



        }
    }
  }

//18.5 is 90 deg turn
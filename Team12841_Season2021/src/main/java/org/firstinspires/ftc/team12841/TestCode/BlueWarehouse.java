package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

@Autonomous(name = "blu warehouse")

public class BlueWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this,RobotHardware.CamX1,RobotHardware.CamY, RobotHardware.CamX2, RobotHardware.CamY, RobotHardware.CamX3, RobotHardware.CamY, RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));
        LiftDrive liftDrive = new LiftDrive(robotHardware);
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPos;
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPosFound = RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER;

        //waitForStart();
        while (!isStarted())
        {
            telemetry.addData("Team Marker Pos", robotHardware.pipeline.markerPos);
            telemetry.update();
            markerPosFound = robotHardware.pipeline.markerPos;
        }
        markerPosFound = robotHardware.pipeline.markerPos;
        //move to warehouse


        if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.LEFT){
            liftDrive.StartAction(.5, 4, 5, true);
            encoderDrive.StartAction(0.5, -12, -12, 5, true);
            encoderDrive.StartAction(0.5, -9, 9, 5, true);
            encoderDrive.StartAction(0.5, -13.2, -13.2, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 11, 11, 5, true);
            encoderDrive.StartAction(0.5, -9.7, 9.7, 5, true);
            encoderDrive.StartAction(1.0, 42, 42, 5, true);
            liftDrive.StartAction(.5, -4, 5, true);

        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER){
            liftDrive.StartAction(.5, 10, 5, true);
            encoderDrive.StartAction(0.5, -12, -12, 5, true);
            encoderDrive.StartAction(0.5, -9, 9, 5, true);
            encoderDrive.StartAction(0.5, -13.2, -13.2, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 11, 11, 5, true);
            encoderDrive.StartAction(0.5, -10, 10, 5, true);
            encoderDrive.StartAction(1.0, 40, 40, 5, true);
            liftDrive.StartAction(.5, -10, 5, true);

        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.RIGHT){
            liftDrive.StartAction(.5, 13.45, 5, true);
            encoderDrive.StartAction(0.5, -30, -30, 5, true);
            encoderDrive.StartAction(0.5, 18, 18, 5, true);
            encoderDrive.StartAction(0.5, -8, 8, 5, true);
            encoderDrive.StartAction(0.5, -15, -15, 5, true);
            robotHardware.InServo.setPosition(.60);
            sleep(600);
            robotHardware.InMotor.setPower(1);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            robotHardware.InServo.setPosition(.8);
            encoderDrive.StartAction(0.5, 11, 11, 5, true);
            encoderDrive.StartAction(0.5, -10, 10, 5, true);
            encoderDrive.StartAction(1.0, 40, 40, 5, true);
            liftDrive.StartAction(.5, -13.45, 5, true);

        }


// might need to be changed with final robot

    }
}

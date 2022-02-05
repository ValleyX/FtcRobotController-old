package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

@Autonomous(name = "red warehouse")

public class RedWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this, RobotHardware.CamX1 + 20 ,RobotHardware.CamY , RobotHardware.CamX2 + 20, RobotHardware.CamY , RobotHardware.CamX3 + 20, RobotHardware.CamY, RobotHardware.cameraSelection.LEFT);
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
        //move to warehouse
        if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.LEFT){
            liftDrive.StartAction(.5, 4, 5, true);
            encoderDrive.StartAction(0.5, -12, -12, 5, true);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);
            encoderDrive.StartAction(0.5, -5, -5, 5, true);
            encoderDrive.StartAction(0.5, -5, 5, 5, true);
            encoderDrive.StartAction(0.5, -10, -10, 5, true);
            encoderDrive.StartAction(0.5, 10, 10, 5, true);
            encoderDrive.StartAction(0.5, 11.5, -11.5, 5, true);
            encoderDrive.StartAction(0.5, -9, -9, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 12, 12, 5, true);
            encoderDrive.StartAction(0.5, 8.5, -8.5, 5, true);
            encoderDrive.StartAction(1.0, 35, 35, 5, true);
            liftDrive.StartAction(.5, -4, 5, true);

        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.CENTER){
            liftDrive.StartAction(.5, 10, 5, true);
            encoderDrive.StartAction(0.5, -12, -12, 5, true);
            encoderDrive.StartAction(0.5, 5, -5, 5, true);
            encoderDrive.StartAction(0.5, -5, -5, 5, true);
            encoderDrive.StartAction(0.5, -5, 5, 5, true);
            encoderDrive.StartAction(0.5, -10, -10, 5, true);
            encoderDrive.StartAction(0.5, 10, 10, 5, true);
            encoderDrive.StartAction(0.5, 11.5, -11.5, 5, true);
            encoderDrive.StartAction(0.5, -9.5, -9.5, 5, true);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            encoderDrive.StartAction(0.5, 12, 12, 5, true);
            encoderDrive.StartAction(0.5, 8.5, -8.5, 5, true);
            encoderDrive.StartAction(1.0, 35, 35, 5, true);
            liftDrive.StartAction(.5, -10, 5, true);

        } else if (markerPosFound == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.RIGHT){
            liftDrive.StartAction(.5, 13.45, 5, true);
            encoderDrive.StartAction(0.5, -30, -30, 5, true);
            encoderDrive.StartAction(0.5, 18.5, 18.5, 5, true);
            encoderDrive.StartAction(0.5, 12, -12, 5, true);
            encoderDrive.StartAction(0.5, -12.5, -12.5, 5, true);
            robotHardware.InServo.setPosition(.60);
            sleep(600);
            robotHardware.InMotor.setPower(.5);
            sleep(600);
            robotHardware.InMotor.setPower(0);
            robotHardware.InServo.setPosition(.8);
            encoderDrive.StartAction(0.5, 12, 12, 5, true);
            encoderDrive.StartAction(0.5, 9, -9, 5, true);
            encoderDrive.StartAction(1.0, 35, 35, 5, true);
            liftDrive.StartAction(.5, -13.45, 5, true);

        }
// might change with final bot

    }
}

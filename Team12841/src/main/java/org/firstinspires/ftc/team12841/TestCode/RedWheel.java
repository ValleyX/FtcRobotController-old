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
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos markerPos;

        //waitForStart();
        while (!isStarted())
        {
            telemetry.addData("Team Marker Pos", robotHardware.pipeline.markerPos);
            telemetry.update();
            markerPos = robotHardware.pipeline.markerPos;
        }
//this all needs reversing:
        //move to wheel
        liftDrive.StartAction(.5, 2.5, 5, true);
        encoderDrive.StartAction(0.5, -5, -5, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, -19, 19, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, 20, 20, 5, true);
        sleep(200);
        encoderDrive.StartAction(.5,17,-17,5,true);
        encoderDrive.StartAction(0.5,4.5, 4.5, 5, true);

        //Below is the turntable spinning motor
        robotHardware.WheelMotor.setPower(-1);
        sleep(4500);
        robotHardware.WheelMotor.setPower(0);



        encoderDrive.StartAction(.5, 7, -7, 5, true);
        encoderDrive.StartAction(.5, -17.3, -17.3, 5, true);
        encoderDrive.StartAction(.5, -4.5, 4.5, 5, true);
        encoderDrive.StartAction(.5, -2, -2, 5, true);
        // might change with final bot
    }
}


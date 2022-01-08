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
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this, RobotHardware.CamX1 + 20 ,RobotHardware.CamY, RobotHardware.CamX2 + 20, RobotHardware.CamY, RobotHardware.CamX3 + 20, RobotHardware.CamY, RobotHardware.cameraSelection.LEFT);
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
        //move to warehouse
        liftDrive.StartAction(.5,3.5,5,true);
        encoderDrive.StartAction(0.5, -16, -16, 5, true);
        encoderDrive.StartAction(.5, 17, -17, 5, true);
        encoderDrive.StartAction(.5, 40, 40, 5, true);
// might change with final bot

    }
}

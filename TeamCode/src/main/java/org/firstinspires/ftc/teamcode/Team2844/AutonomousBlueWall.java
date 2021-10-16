package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Drivers.RotateToHeading;

@Autonomous (name="BlueWall")
// hello
@Disabled
public class AutonomousBlueWall extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this, 100, 140, RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot);
        RotatePrecise rotatePrecise =  new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);

        RobotHardware.SkystoneDeterminationPipeline.RingPosition path = robot.pipeline.position;

        while (!isStarted())
        {
            path = robot.pipeline.position;
            telemetry.addData("Number of Rings", robot.pipeline.position);
            telemetry.update();
        }
        robot.switchableWebcam.stopStreaming();
        // placement: right wheels on line

        waitForStart();

        System.out.println("path value = " + path);

        final double WHITELINE_DISTANCE = 68; //72
        final double BOXLENGTH = 27; //22.75
        final double DISTANCETO_BOXB = 7; //9
        final double EXTRALENGTH = 9;


        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-7.5, 0, 10, true);
            rotateToHeading.DoIt(-35);
            // drops wobble goal, already on line
        }

        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.ONE) // Square B, 1 ring
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH, 0, 10, true);
            rotateToHeading.DoIt(90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXB, 90, 10, true);
            sleep(2000);
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXB, 90, 10, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH+2, 0, 5, true);
        }

        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH+EXTRALENGTH, 0, 10, true);
            rotateToHeading.DoIt(-30);
            sleep(2000);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH-EXTRALENGTH, 0, 10, true);
        }
    }
}

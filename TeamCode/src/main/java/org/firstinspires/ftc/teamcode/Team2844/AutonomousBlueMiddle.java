package org.firstinspires.ftc.teamcode.Team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;

@Autonomous (name="BlueMiddle")
// hello
@Disabled
public class AutonomousBlueMiddle extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this, 250, 140, RobotHardware.cameraSelection.RIGHT);
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
        // placement: black lines

        waitForStart();

        System.out.println("path value = " + path);

        final double WHITELINE_DISTANCE = 68; //72
        final double BOXLENGTH = 27; //22.75
        final double DISTANCETO_BOXB = 11; //9
        final double EXTRALENGTH = 9;
        final double DISTANCETO_BOXAC = 15;
        final double DISTANCETO_RINGS = 17;
        final double INITIAL_DISTANCE = 15;

        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            // grab wobble goal
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+EXTRALENGTH, 0, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXAC, -90, 5, true);
            sleep(2000); // release
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
        }

        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.ONE) // Square B, 1 ring
        {
            // getting rings
            encoderDriveHeading.StartAction(0.8, INITIAL_DISTANCE, 0, 10, true);
            rotateToHeading.DoIt(-30);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_RINGS, -30, 10, true);
            // intake ring
            sleep(2000);
            // shoot ring
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_RINGS, -30, 10, true);
            rotateToHeading.DoIt(0);

            //delivering wobble goal
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-INITIAL_DISTANCE, 0, 10, true);
            rotateToHeading.DoIt(30);
            encoderDriveHeading.StartAction(0.8, BOXLENGTH+8, 30, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXB, -90, 5, true);
            sleep(2000); // release
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXB, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH+4, 0, 10, true);
        }

        if (path == RobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            // getting rings
            // grab wobble goal
            encoderDriveHeading.StartAction(0.8, INITIAL_DISTANCE, 0, 10, true);
            rotateToHeading.DoIt(-30);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_RINGS, -30, 10, true);
            // intake ring
            sleep(2000);
            // shoot ring
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_RINGS, -30, 10, true);
            rotateToHeading.DoIt(0);

            //delivering wobble goal
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH+EXTRALENGTH-1, 0, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXAC, -90, 5, true);
            sleep(2000); // release
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH-15, 0, 10, true);
        }
    }
}

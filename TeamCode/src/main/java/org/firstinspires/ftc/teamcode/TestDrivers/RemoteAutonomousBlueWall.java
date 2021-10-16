package org.firstinspires.ftc.teamcode.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.EncoderFourWheelDriveHeading;
import org.firstinspires.ftc.teamcode.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Drivers.RotatePreciseFourWheelDrive;
import org.firstinspires.ftc.teamcode.Drivers.RotateToHeadingFourWheelDrive;

@Autonomous (name="RemoteBlueWall")
@Disabled
public class RemoteAutonomousBlueWall extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 5, 135, MandoRobotHardware.cameraSelection.LEFT);
        //EncoderFourWheelDriveHeading encoderDrive = new EncoderFourWheelDriveHeading(robot);
        EncoderFourWheelDriveHeading encoderDriveHeading = new EncoderFourWheelDriveHeading(robot);
        RotatePreciseFourWheelDrive rotatePrecise =  new RotatePreciseFourWheelDrive(robot);
        RotateToHeadingFourWheelDrive rotateToHeading = new RotateToHeadingFourWheelDrive(robot, rotatePrecise);

        MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition path = robot.pipeline.position;

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


        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-4, 0, 10, true); //-7.5
            rotateToHeading.DoIt(-35);
            // drops wobble goal
            encoderDriveHeading.StartAction(0.8, -11.5, -35, 5, true);
            rotateToHeading.DoIt(165);
            encoderDriveHeading.StartAction(0.8, 30, 165, 10, true);
            sleep(2000); // pick up wobble goal
            encoderDriveHeading.StartAction(0.8, -30, 165, 10, true);
            rotateToHeading.DoIt(-15);
            encoderDriveHeading.StartAction(0.8, 8, -15, 5, true);

        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.ONE) // Square B, 1 ring
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH, 0, 10, true);
            rotateToHeading.DoIt(90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXB, 90, 10, true);
            sleep(2000);
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXB, 90, 10, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH+2, 0, 5, true);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH+EXTRALENGTH, 0, 10, true);
            rotateToHeading.DoIt(-30);
            sleep(2000);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH-EXTRALENGTH, 0, 10, true);
        }
    }
}

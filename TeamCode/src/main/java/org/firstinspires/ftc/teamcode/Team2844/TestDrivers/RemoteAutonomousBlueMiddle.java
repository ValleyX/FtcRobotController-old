package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderFourWheelDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePreciseFourWheelDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeadingFourWheelDrive;

@Autonomous (name="RemoteBlueMiddle")
public class RemoteAutonomousBlueMiddle extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 260, 170, MandoRobotHardware.cameraSelection.RIGHT);
        EncoderFourWheelDriveHeading encoderDriveHeading = new EncoderFourWheelDriveHeading(robot);
        RotatePreciseFourWheelDrive rotatePrecise =  new RotatePreciseFourWheelDrive(robot);
        RotateToHeadingFourWheelDrive rotateToHeading = new RotateToHeadingFourWheelDrive(robot, rotatePrecise);

        MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition path = robot.pipeline.position;

        // initialize wobble arm to hold wobble goal, and bucket and sweeper to be in the right spot
        robot.wobbleServo.setPosition(robot.wobbleUp);
        sleep(500);
        robot.clasper.setPosition(robot.clasperClosed);
        sleep(500);
        robot.nucketyServo.setPosition(robot.nucketyUp);
        sleep(500);
        robot.sweepyServo.setPosition(robot.sweepyOut);
        sleep(500);

        while (!isStarted())
        {
            path = robot.pipeline.position;
            telemetry.addData("Number of Rings", robot.pipeline.position);
            telemetry.update();
        }

        robot.switchableWebcam.stopStreaming();
        // placement: black lines

        System.out.println("path value = " + path);

        // variables
        final double WHITELINE_DISTANCE = 68;
        final double BOXLENGTH = 27;
        final double DISTANCETO_BOXAC = 12;
        final double INITIAL_MOVEMENT = 50;

        //robot.intake.setPower(0.8);

        // shooting 3 rings (same for all positions)
        robot.backshot.setPower(0.635);
        robot.frontshot.setPower(0.635);
        // drive up to white line to shoot (more accurate)
        encoderDriveHeading.StartAction(0.7, INITIAL_MOVEMENT, 0, 5, true);
        // slightly off center, so turn to make up for it
        rotateToHeading.DoIt(-2);
        sleep(1000);
        robot.sweepyServo.setPosition(robot.sweepyPush);
        sleep(500);
        robot.sweepyServo.setPosition(robot.sweepyOut);
        sleep(2000);
        robot.sweepyServo.setPosition(robot.sweepyPush);
        sleep(500);
        robot.sweepyServo.setPosition(robot.sweepyOut);
        sleep(2000);
        robot.sweepyServo.setPosition(robot.sweepyPush);
        sleep(500);
        robot.sweepyServo.setPosition(robot.sweepyOut);
        sleep(1000);
        rotateToHeading.DoIt(0);
        robot.backshot.setPower(0.0);
        robot.frontshot.setPower(0.0);

        // continues on depending on how many rings it saw
        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            // drive up the rest of the way, and drop off wobble goal, and come back to the white line
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-INITIAL_MOVEMENT+8, 0, 8, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXAC, -90, 5, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            sleep(1000);
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -9, 0, 5, true);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.ONE) // Square B, 1 ring
        {
            // drive up in front of the box, and drop off the wobble goal (ends on the white line so no need to back up)
            encoderDriveHeading.StartAction(0.6, WHITELINE_DISTANCE-INITIAL_MOVEMENT+5, 0, 10, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            sleep(1000);
            rotateToHeading.DoIt(0);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            // drive up to farthest box, place wobble goal, and drive back to white line
            encoderDriveHeading.StartAction(0.7, WHITELINE_DISTANCE+BOXLENGTH-INITIAL_MOVEMENT+29, 0, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.7, DISTANCETO_BOXAC, -90, 5, true);
            rotateToHeading.DoIt(-70);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            sleep(1000);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.6, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.6, -BOXLENGTH-15, 0, 10, true);
        }
    }
}

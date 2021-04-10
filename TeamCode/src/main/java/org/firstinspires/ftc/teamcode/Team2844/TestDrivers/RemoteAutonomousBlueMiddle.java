package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        MandoRobotHardware robot = new MandoRobotHardware(this, 255, 170, MandoRobotHardware.cameraSelection.RIGHT); //265, 160
        EncoderFourWheelDriveHeading encoderDriveHeading = new EncoderFourWheelDriveHeading(robot);
        RotatePreciseFourWheelDrive rotatePrecise =  new RotatePreciseFourWheelDrive(robot);
        RotateToHeadingFourWheelDrive rotateToHeading = new RotateToHeadingFourWheelDrive(robot, rotatePrecise);
        ElapsedTime runtime_ = new ElapsedTime();

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

        // display number of rings the camera sees before start
        while (!isStarted())
        {
            path = robot.pipeline.position;
            telemetry.addData("Number of Rings", robot.pipeline.position);
            telemetry.update();
        }

        // turn camera off
        robot.switchableWebcam.stopStreaming();
        // placement: black lines

        System.out.println("path value = " + path);

        // variables
        final double WHITELINE_DISTANCE = 68;
        final double BOXLENGTH = 27;
        final double DISTANCETO_BOXAC = 12;
        final double INITIAL_MOVEMENT = 55;

        final double DISTANCE_FROM_WOBBLE = 4.8;
        int seconds = 15;

        robot.intake.setPower(1.0);

        // shooting 3 rings (same for all positions)
        robot.backshot.setPower(0.35); //0.56
        robot.frontshot.setPower(0.35);
        int rpsCount = 1010; //1280
        // drive up to white line to shoot (more accurate)
        encoderDriveHeading.StartAction(0.7, INITIAL_MOVEMENT, 0, 5, true);
        // launch rings to hit powershot
        double changeHeading = 5;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(rpsCount, 1); //1440
        changeHeading+=4.5;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(rpsCount, 1); //1440
        changeHeading+=5; //6
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(rpsCount, 1); //1440
        robot.backshot.setPower(0.0);
        robot.frontshot.setPower(0.0);
        robot.nucketyServo.setPosition(robot.nucketyDown);
        sleep(500);

        // continues on depending on how many rings it saw
        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            // drive up the rest of the way, and drop off wobble goal, and come back to the white line
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-INITIAL_MOVEMENT+11, 0, 8, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXAC, -90, 5, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXAC-5, -90, 5, true);

            // driving back for second wobble goal
            rotateToHeading.DoIt(180);
            encoderDriveHeading.StartAction(0.8, 61, 180, 5, true); //55
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            rotateToHeading.DoIt(-90);

            // driving up to second wobble goal
            while ((robot.distance.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_WOBBLE) && runtime_.milliseconds() < seconds)
            {
                double speed = 0.5;
                robot.rightBackDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.leftBackDrive.setPower(speed);
                robot.leftFrontDrive.setPower(speed);
            }
            robot.rightBackDrive.setPower(0.0);
            robot.rightFrontDrive.setPower(0.0);
            robot.leftBackDrive.setPower(0.0);
            robot.leftFrontDrive.setPower(0.0);
            encoderDriveHeading.StartAction(0.8, 6, -90, 5, true);
            rotatePrecise.RotatePrecise(15, 2, 0.4, 0.1, 5);
            sleep(300);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            encoderDriveHeading.StartAction(0.8, -6, -90, 5, true);
            rotateToHeading.DoIt(180);

            // driving back to place second wobble goal
            encoderDriveHeading.StartAction(0.8, -61, 180, 5, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_BOXAC, -90, 5, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.8, -5, 0, 5, true);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.ONE) // Square B, 1 ring
        {
            // drive up in front of the box, and drop off the wobble goal (ends on the white line so no need to back up)
            //robot.intake.setPower(-1.0);
            encoderDriveHeading.StartAction(0.6, WHITELINE_DISTANCE-INITIAL_MOVEMENT+7+11, 0, 10, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            rotateToHeading.DoIt(0);
            //robot.clasper.setPosition(robot.clasperClosed);
            //robot.wobbleServo.setPosition(robot.wobbleUp);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            // drive up to farthest box, place wobble goal, and drive back to white line
            encoderDriveHeading.StartAction(0.7, WHITELINE_DISTANCE+BOXLENGTH-INITIAL_MOVEMENT+27+20, 0, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.7, DISTANCETO_BOXAC, -90, 5, true);
            rotateToHeading.DoIt(-70);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.6, -DISTANCETO_BOXAC-20, -90, 5, true);
            rotateToHeading.DoIt(0);
            encoderDriveHeading.StartAction(0.6, -BOXLENGTH-15, 0, 10, true);
            //robot.clasper.setPosition(robot.clasperClosed);
            //robot.wobbleServo.setPosition(robot.wobbleUp);
        }

        robot.intake.setPower(0.0);

        while (opModeIsActive())
        {
            sleep(50);
        }

    }
}
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
// hello
public class RemoteAutonomousBlueMiddle extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 260, 170, MandoRobotHardware.cameraSelection.RIGHT);
        //EncoderFourWheelDriveHeading encoderDrive = new EncoderFourWheelDriveHeading(robot);
        EncoderFourWheelDriveHeading encoderDriveHeading = new EncoderFourWheelDriveHeading(robot);
        RotatePreciseFourWheelDrive rotatePrecise =  new RotatePreciseFourWheelDrive(robot);
        RotateToHeadingFourWheelDrive rotateToHeading = new RotateToHeadingFourWheelDrive(robot, rotatePrecise);
        // EasyOpenCVExample RingDetection = new EasyOpenCVExample();

        //robot.pipeline.getAnalysis();


        //int location = robot.pipeline.getAnalysis();

        MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition path = robot.pipeline.position;

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



        //waitForStart();

        //telemetry.addData("path value = ", path);
        System.out.println("path value = " + path);

        final double WHITELINE_DISTANCE = 68;
        final double BOXLENGTH = 27;
        final double DISTANCETO_BOXB = 12; //11
        final double EXTRALENGTH = 0; //9
        final double DISTANCETO_BOXAC = 12; //15
        final double DISTANCETO_RINGS = 17;
        final double INITIAL_DISTANCE = 15;
        final double INITIAL_MOVEMENT = 48;

        robot.intake.setPower(0.8);

        robot.backshot.setPower(0.60);
        robot.frontshot.setPower(0.60);
        encoderDriveHeading.StartAction(0.7, INITIAL_MOVEMENT, 0, 5, true);
        //sleep(2000);
        rotateToHeading.DoIt(0);
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
        robot.backshot.setPower(0.0);
        robot.frontshot.setPower(0.0);
        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+EXTRALENGTH-INITIAL_MOVEMENT+8, 0, 8, true);
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
            // getting rings

            /*
            encoderDriveHeading.StartAction(0.8, INITIAL_DISTANCE, 0, 10, true);
            rotateToHeading.DoIt(-30);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_RINGS, -30, 10, true);
            // intake ring
            sleep(2000);
            // shoot ring
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_RINGS, -30, 10, true);
            rotateToHeading.DoIt(0);
             */

            //delivering wobble goal
            /*
            encoderDriveHeading.StartAction(0.6, WHITELINE_DISTANCE-INITIAL_DISTANCE-INITIAL_MOVEMENT+6, 0, 10, true);
            rotateToHeading.DoIt(30);
            encoderDriveHeading.StartAction(0.6, BOXLENGTH+20, 30, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.6, DISTANCETO_BOXB, -90, 5, true);

             */
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
            //encoderDriveHeading.StartAction(0.6, -DISTANCETO_BOXB+5, -90, 5, true);
            rotateToHeading.DoIt(0);
            //encoderDriveHeading.StartAction(0.6, -BOXLENGTH+2, 0, 10, true); //+4
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            // getting rings

            /*
            encoderDriveHeading.StartAction(0.8, INITIAL_DISTANCE, 0, 10, true);
            rotateToHeading.DoIt(-30);
            encoderDriveHeading.StartAction(0.8, DISTANCETO_RINGS, -30, 10, true);
            // intake ring
            sleep(2000);
            // shoot ring
            encoderDriveHeading.StartAction(0.8, -DISTANCETO_RINGS, -30, 10, true);
            rotateToHeading.DoIt(0);

             */

            //delivering wobble goal
            encoderDriveHeading.StartAction(0.7, WHITELINE_DISTANCE+BOXLENGTH+EXTRALENGTH-INITIAL_MOVEMENT+33, 0, 10, true);
            rotateToHeading.DoIt(-90);
            encoderDriveHeading.StartAction(0.7, DISTANCETO_BOXAC, -90, 5, true);
            rotateToHeading.DoIt(-60);
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

            robot.wobbleServo.setPosition(robot.wobbleDown);
        }
    }
}

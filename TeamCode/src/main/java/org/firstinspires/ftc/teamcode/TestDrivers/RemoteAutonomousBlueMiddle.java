package org.firstinspires.ftc.teamcode.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.EncoderFourWheelDriveHeading;
import org.firstinspires.ftc.teamcode.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Drivers.RotatePreciseFourWheelDrive;
import org.firstinspires.ftc.teamcode.Drivers.RotateToHeadingFourWheelDrive;

@Autonomous (name="RemoteBlueMiddle")
public class RemoteAutonomousBlueMiddle extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 45, 130, MandoRobotHardware.cameraSelection.LEFT); //62,138
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
        /* test distance sensor
        {
            final double DISTANCE_FROM_WOBBLE = 4.8;
            double seconds = 3.0;

            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(800);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(800);
            runtime_.reset();
            // driving up to second wobble goal
            telemetry.addData("distance", robot.distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("seconds", runtime_.seconds());
            telemetry.update();
            while ((robot.distance.getDistance(DistanceUnit.INCH) > DISTANCE_FROM_WOBBLE) && runtime_.seconds() < seconds) {
                double speed = 0.3;
                robot.rightBackDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.leftBackDrive.setPower(speed);
                robot.leftFrontDrive.setPower(speed);
                telemetry.addData("distance", robot.distance.getDistance(DistanceUnit.INCH));
                telemetry.addData("seconds", runtime_.seconds());
                telemetry.update();
            }
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            while (!isStarted()) {

            }
        }

         */
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

        final double DISTANCE_FROM_WOBBLE = 4.6;//4.8
        int seconds = 3;

        // continues on depending on how many rings it saw
        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.NONE) // Square A, 0 rings
        {
            robot.intake.setPower(1.0);

            // shooting 3 rings (same for all positions)
            robot.backshot.setPower(0.35); //0.56
            robot.frontshot.setPower(0.35);
            int rpsCount = 1010; //1280
            // drive up to white line to shoot (more accurate)
            encoderDriveHeading.StartAction(0.7, INITIAL_MOVEMENT, 0, 5, true);
            // launch rings to hit powershot
            double changeHeading = 5;
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5); //0.65
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            changeHeading+=4.5;
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5);
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            changeHeading+=5; //6
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5);
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            robot.backshot.setPower(0.0);
            robot.frontshot.setPower(0.0);
            robot.nucketyServo.setPosition(robot.nucketyDown);
            sleep(500);

            // drive up the rest of the way, and drop off wobble goal, and come back to the white line
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE-INITIAL_MOVEMENT+7, 0, 8, true); //11
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
            encoderDriveHeading.StartAction(0.8, 59, 180, 5, true); //61
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(800);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(800);
            rotateToHeading.DoIt(-90);

            /*
            runtime_.reset();
            // driving up to second wobble goal
            while ((robot.distance.getDistance(DistanceUnit.INCH) > DISTANCE_FROM_WOBBLE) && runtime_.seconds() < seconds)
            {
                double speed = 0.3;
                robot.rightBackDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.leftBackDrive.setPower(speed);
                robot.leftFrontDrive.setPower(speed);
            }
            robot.rightBackDrive.setPower(0.0);
            robot.rightFrontDrive.setPower(0.0);
            robot.leftBackDrive.setPower(0.0);
            robot.leftFrontDrive.setPower(0.0);
             */

            encoderDriveHeading.StartAction(0.8, 5, -90, 5, true);
            rotatePrecise.RotatePrecise(12, 2, 0.4, 0.1, 5); //15
            sleep(300);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(800);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            encoderDriveHeading.StartAction(0.8, -6, -90, 5, true);
            rotateToHeading.DoIt(180);

            // driving back to place second wobble goal
            encoderDriveHeading.StartAction(0.8, -56, 180, 5, true); //56
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
            robot.backshot.setPower(0.56);
            robot.frontshot.setPower(0.56);
            sleep(500);
            robot.sweepyServo.setPosition(robot.sweepyPush);
            sleep(500);
            robot.sweepyServo.setPosition(robot.sweepyOut);
            sleep(500);
            robot.nucketyServo.setPosition(robot.nucketyDown);

            robot.intake.setPower(-1.0);

            // shooting 3 rings (same for all positions)
            robot.backshot.setPower(0.35); //0.56
            robot.frontshot.setPower(0.35);
            int rpsCount = 1010; //1280
            // drive up to white line to shoot (more accurate)
            encoderDriveHeading.StartAction(1.0, INITIAL_MOVEMENT, 0, 5, true);
            sleep(1500);
            // launch rings to hit powershot
            robot.nucketyServo.setPosition(robot.nucketyUp);
            double changeHeading = 5;
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5);
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            changeHeading+=4.5;
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5);
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            changeHeading+=5; //6
            rotateToHeading.DoItSpecify(changeHeading, 0.75, 0.41, 0.1, 5);
            robot.ThreeRingLaunch(rpsCount, 1); //1440
            robot.backshot.setPower(0.0);
            robot.frontshot.setPower(0.0);
            robot.nucketyServo.setPosition(robot.nucketyDown);
            sleep(500);

            // drive up the rest of the way, and drop off wobble goal, and come back to the white line
            encoderDriveHeading.StartAction(1.0, WHITELINE_DISTANCE-INITIAL_MOVEMENT+12, 0, 10, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.clasperMid);
            rotateToHeading.DoIt(0);

            // driving back for second wobble goal
            //rotateToHeading.DoIt(180);
            encoderDriveHeading.StartAction(1.0, -42, 0, 5, true); //40
            rotateToHeading.DoIt(-140); //-145
            encoderDriveHeading.StartAction(0.8, 10, -140, 5, true); //55
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            encoderDriveHeading.StartAction(0.45, 5, -140, 5, true); //55
            sleep(500);
            rotateToHeading.DoIt(-130);
            sleep(500);
            robot.clasper.setPosition(robot.clasperClosed);
            sleep(1000);
            robot.wobbleServo.setPosition(robot.wobbleUp);
            sleep(1000);
            rotateToHeading.DoIt(180);
            //rotateToHeading.DoIt(-90);

            // driving back to place second wobble goal
            encoderDriveHeading.StartAction(0.8, -40, 180, 5, true);
            rotateToHeading.DoIt(15);
            encoderDriveHeading.StartAction(0.6, 9, 15, 5, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(1000);
            //robot.wobbleServo.setPosition(robot.clasperMid);
            encoderDriveHeading.StartAction(0.8, -8, 0, 5, true);
        }

        if (path == MandoRobotHardware.SkystoneDeterminationPipeline.RingPosition.FOUR) // Square C, 4 rings
        {
            // shoot rings
            robot.frontshot.setPower(0.405);//39
            robot.backshot.setPower(0.405);
            sleep(500);
            //encoderDriveHeading.StartAction(0.8, 30, 0, 5, true);
            encoderDriveHeading.StartAction(1.0, 29, 0, 5, true);
            rotateToHeading.DoIt(-4);
            robot.ThreeRingLaunch(1390, 3);
            rotateToHeading.DoIt(-6);
            robot.nucketyServo.setPosition(robot.nucketyDown);
            robot.sweepyServo.setPosition(robot.sweepyOut);
            robot.intake.setPower(-1.0);
            //encoderDriveHeading.StartAction(0.15, 10, -6, 5, true);

            // drive up to intake ring stack
            robot.nucketyServo.setPosition(robot.nucketyDown);
            robot.sweepyServo.setPosition(robot.sweepyOut);
            robot.backshot.setPower(0.39);
            robot.frontshot.setPower(0.39);
            sleep(500);
            robot.intake.setPower(-1.0);
            encoderDriveHeading.StartAction(0.2, 15.5, -6, 5, true);
            sleep(4000);
            rotateToHeading.DoIt(-4);
            robot.intake.setPower(1.0);
            robot.nucketyServo.setPosition(robot.nucketyUp);
            sleep(1000);
            robot.ThreeRingLaunch(1360, 2);

            // drive farther to intake remaining rings
            robot.intake.setPower(-1.0);
            robot.frontshot.setPower(0.38);
            robot.backshot.setPower(0.38);
            robot.nucketyServo.setPosition(robot.nucketyDown);
            encoderDriveHeading.StartAction(0.2, 11, -4, 5, true); //13
            sleep(2000);
            /*
            encoderDriveHeading.StartAction(0.2, 7, -4, 5, true);
            sleep(500);
            encoderDriveHeading.StartAction(0.7, -13, -4, 5, true);
            
             */
            robot.intake.setPower(1.0);
            robot.nucketyServo.setPosition(robot.nucketyUp);
            sleep(1000);
            robot.ThreeRingLaunch(1320, 2);
            robot.nucketyServo.setPosition(robot.nucketyDown);

            // drive up to farthest box, place wobble goal, and drive back to white line (10 degree angle approach to save time)
            rotateToHeading.DoIt(-13);
            encoderDriveHeading.StartAction(1.0, 51, -13, 10, true);
            robot.wobbleServo.setPosition(robot.wobbleDown);
            sleep(1000);
            robot.clasper.setPosition(robot.clasperOpen);
            sleep(600);
            //robot.wobbleServo.setPosition(robot.clasperMid);
            encoderDriveHeading.StartAction(1.0, -38, -13, 10, true);
            rotateToHeading.DoIt(0);

            /*
            // drive up to farthest box, place wobble goal, and drive back to white line
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

             */
        }

        robot.intake.setPower(0.0);

        while (opModeIsActive())
        {
            sleep(50);
        }
    }
}
package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadRunner1Score extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robotHardware = new RobotHardware(this, true);
        LiftHardware liftHardware = new LiftHardware(robotHardware, this);

        waitForStart();
        if(isStopRequested()) return;


        Trajectory place1 = drive.trajectoryBuilder(new Pose2d(40.5, 63.5, Math.toRadians(0)))
                .splineTo(new Vector2d(54, 50), Math.toRadians(45))
                .build();

        Trajectory forwardPlace1 = drive.trajectoryBuilder(place1.end())
                .forward(13)
                .build();

        Trajectory backUp1 = drive.trajectoryBuilder(forwardPlace1.end())
                .back(13)
                .build();

        TrajectorySequence turn1 = drive.trajectorySequenceBuilder(backUp1.end())
                .turn(Math.toRadians(-135))
                .build();

        Trajectory pickup1 = drive.trajectoryBuilder(turn1.end())
                .splineTo(new Vector2d(50.0, 34), Math.toRadians(-90))
                .build();

        Trajectory backUp2 = drive.trajectoryBuilder(pickup1.end())
                .back(8)
                .build();

        Trajectory place2 = drive.trajectoryBuilder(backUp2.end())
                .splineTo(new Vector2d(54, 50), Math.toRadians(45))
                .build();

        Trajectory forwardPlace2 = drive.trajectoryBuilder(place2.end())
                .forward(10)
                .build();

        Trajectory backUp3 = drive.trajectoryBuilder(forwardPlace2.end())
                .back(10)
                .build();

        drive.setPoseEstimate(new Pose2d(40.5, 63.5, Math.toRadians(0)));

        //place the preloaded into the basket
        liftHardware.closeClaw();                            //clamp down on preloaded sample
        liftHardware.moveYSlides(30, 1);          //start moving the slides up while the robot drives
        drive.followTrajectory(place1);                     //move ten "inches" away from the basket
        liftHardware.moveYSlidesAuto(44, 1);      //move the slides all the way up
        drive.followTrajectory(forwardPlace1);              // move over the basket
        liftHardware.openClaw();                            //let go of sample
        sleep(500);
        drive.followTrajectory(backUp1);                    //move back so the slides don't go down on the basket
        liftHardware.moveYSlidesAuto(30, 1);      //move the slides down so the robot doesn't tip over
        liftHardware.moveYSlides(0, 1);           //move the slides all the way down while moving the robot
        drive.followTrajectorySequence(turn1);              //turn towards the samples on the floor so that the driver doesn't have to reset the imu
        sleep(3000);
        liftHardware.moveYSlidesAuto(0, 1);
    }
}

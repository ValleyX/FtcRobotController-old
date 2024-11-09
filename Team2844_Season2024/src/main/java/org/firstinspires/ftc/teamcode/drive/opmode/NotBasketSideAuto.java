package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Autonomous(group = "drive") //makes it an autonomous program
public class NotBasketSideAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.bucketServo); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftReset liftReset = new LiftReset(m_liftSub,this);

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;

        //make trajectories
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0, 0)) //start position
//                .splineTo(new Vector2d(0, 0), 0) //finishing position
//                .build(); //builds command

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(7, -60, Math.toRadians(90)))
                .forward(24)
                .build();

        Trajectory back = drive.trajectoryBuilder(new Pose2d(12, -36, Math.toRadians(90)))
                .forward(-24)
                .build();

        Trajectory splineToSamples = drive.trajectoryBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(36, -12), Math.toRadians(90))
                .forward(6)
                .build();

        Trajectory orient1 = drive.trajectoryBuilder(splineToSamples.end())
                .strafeTo(new Vector2d(48, -6))
                .build();

        Trajectory score1A = drive.trajectoryBuilder(orient1.end())
                .back(52)
                .build();

        Trajectory score1B = drive.trajectoryBuilder(score1A.end())
                .forward(52)
                .build();

        Trajectory orient2 = drive.trajectoryBuilder(score1B.end())
                .strafeTo(new Vector2d(60, -6))
                .build();

        Trajectory score2A = drive.trajectoryBuilder(orient2.end())
                .back(50)
                .build();

        Trajectory score2B = drive.trajectoryBuilder(score2A.end())
                .forward(50)
                .build();

        Trajectory orient3 = drive.trajectoryBuilder(score2B.end())
                .strafeTo(new Vector2d(66, -6))
                .build();

        Trajectory score3 = drive.trajectoryBuilder(orient3.end())
                .back(46)
                .build();


        //set start pos
        drive.setPoseEstimate(new Pose2d(7, -60, Math.toRadians(90)));

        sleep(500);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(50);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(200);

        drive.followTrajectory(back);
        sleep(50);

        //resets lift
        liftReset.initialize();
        sleep(100);

        drive.followTrajectory(splineToSamples);
        sleep(50);
        drive.followTrajectory(orient1);
        sleep(50);
        drive.followTrajectory(score1A);
        sleep(50);
        drive.followTrajectory(score1B);
        sleep(50);
        drive.followTrajectory(orient2);
        sleep(50);
        drive.followTrajectory(score2A);
        sleep(50);
        drive.followTrajectory(score2B);
        sleep(50);
        drive.followTrajectory(orient3);
        sleep(50);
        drive.followTrajectory(score3);


    }
}

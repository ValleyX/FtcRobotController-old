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
 */
@Autonomous
public class RoadRunnerQueenCreek extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robotHardware = new RobotHardware(this, true);
        LiftHardware liftHardware = new LiftHardware(robotHardware, this);


        waitForStart();
        if (isStopRequested()) return;

        Trajectory place1 = drive.trajectoryBuilder(new Pose2d(31.5, 63.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();

        TrajectorySequence rotateToPlace = drive.trajectorySequenceBuilder(place1.end())
                .turn(Math.toRadians(49))
                .build();

        TrajectorySequence rotate1 = drive.trajectorySequenceBuilder(rotateToPlace.end())
                .turn(Math.toRadians(21.5))
                .build();

        Trajectory forward1 = drive.trajectoryBuilder(rotate1.end())
                .forward(5.2)
                .build();

        Trajectory backward1 = drive.trajectoryBuilder(forward1.end())
                .back(5.2)
                .build();

        TrajectorySequence rotatePlace1 = drive.trajectorySequenceBuilder(backward1.end())
                .turn(Math.toRadians(-21.5))
                .build();

        TrajectorySequence rotate2 = drive.trajectorySequenceBuilder(rotatePlace1.end())
                .turn(Math.toRadians(38))
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(rotate2.end())
                .forward(3.25)
                .build();

        Trajectory backward2 = drive.trajectoryBuilder(forward2.end())
                .back(3.25)
                .build();

        TrajectorySequence rotatePlace2 = drive.trajectorySequenceBuilder(backward2.end())
                .turn(Math.toRadians(-38))
                .build();

        TrajectorySequence rotate3 = drive.trajectorySequenceBuilder(rotatePlace2.end())
                .turn(Math.toRadians(60))
                .build();


        Trajectory forward3 = drive.trajectoryBuilder(rotate3.end())
                .forward(4.75)
                .build();

        Trajectory backward3 = drive.trajectoryBuilder(forward3.end())
                .back(4.75)
                .build();

        TrajectorySequence rotatePlace3 = drive.trajectorySequenceBuilder(backward3.end())
                .turn(Math.toRadians(-60))
                .build();

        TrajectorySequence endTurn = drive.trajectorySequenceBuilder(rotatePlace3.end())
                .turn(Math.toRadians(45))
                .build();

        drive.setPoseEstimate(new Pose2d(31.5, 63.5, Math.toRadians(180)));

        liftHardware.closeClaw();
        sleep(200);
        liftHardware.movePivotAuto(120, 1);
        liftHardware.moveExtend(20, 1);
        liftHardware.movePivot(200, 1);
        drive.followTrajectory(place1);
        drive.followTrajectorySequence(rotateToPlace);
        robotHardware.clawWrist.setPosition(0);
        liftHardware.movePivotAuto(200, 1);
        liftHardware.moveExtendAuto(20, 1);
        liftHardware.movePivotAuto(217, 1);
        sleep(400);
        liftHardware.openClaw();
        sleep(200);

        robotHardware.clawWrist.setPosition(0.4);
        liftHardware.movePivot(90, 1);
        drive.followTrajectorySequence(rotate1);
        drive.followTrajectory(forward1);
        sleep(100);
        liftHardware.movePivotAuto(liftHardware.getMinAngle(), 0.25);
        liftHardware.closeClaw();
        sleep(200);
        liftHardware.movePivot(200, 0.75);
        drive.followTrajectory(backward1);
        robotHardware.clawWrist.setPosition(0);
        drive.followTrajectorySequence(rotatePlace1);
        sleep(100);
        liftHardware.movePivotAuto(217, 1);
        sleep(400);
        liftHardware.openClaw();
        sleep(200);


        robotHardware.clawWrist.setPosition(0.4);
        liftHardware.movePivot(90, 1);
        drive.followTrajectorySequence(rotate2);
        drive.followTrajectory(forward2);
        liftHardware.movePivotAuto(liftHardware.getMinAngle(), 0.25);
        liftHardware.closeClaw();
        sleep(200);
        liftHardware.movePivot(195, 1);
        robotHardware.clawWrist.setPosition(0);
        drive.followTrajectory(backward2);
        drive.followTrajectorySequence(rotatePlace2);
        liftHardware.movePivotAuto(217, 0.6);
        sleep(400);
        liftHardware.openClaw();
        sleep(200);


        robotHardware.clawWrist.setPosition(0.4);
        liftHardware.movePivotAuto(90, 1);
        drive.followTrajectorySequence(rotate3);
        drive.followTrajectory(forward3);
        liftHardware.movePivotAuto(liftHardware.getMinAngle(), 0.25);
        liftHardware.closeClaw();
        sleep(200);
        drive.followTrajectory(backward3);
        liftHardware.movePivot(195, 1);
        robotHardware.clawWrist.setPosition(0);
        drive.followTrajectorySequence(rotatePlace3);
        liftHardware.movePivotAuto(217, 0.6);
        sleep(400);
        liftHardware.openClaw();
        sleep(200);

        liftHardware.movePivot(90, 1);
        liftHardware.moveExtendAuto(0, 1);
        liftHardware.movePivotAuto(30, 1);
        drive.followTrajectorySequence(endTurn);
    }
}
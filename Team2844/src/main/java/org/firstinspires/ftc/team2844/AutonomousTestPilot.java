package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Autonomous(name="AutoTest")
public class AutonomousTestPilot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        EncoderDriveMecha mechaDrive = new EncoderDriveMecha(robot);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);

        RobotHardware.PowerPlayPipeline.MarkerPosition alphaColor = RobotHardware.PowerPlayPipeline.MarkerPosition.Green;


        while(opModeInInit()){
            //telemetry.addData("red value", robotHardware_.pipeline.avgLeftRed);
            //telemetry.addData("blue value", robotHardware_.pipeline.avgLeftBlue);

            telemetry.addData("r value", robot.pipeline.avgLeftR);
            telemetry.addData("g value", robot.pipeline.avgLeftG);
            telemetry.addData("b value", robot.pipeline.avgLeftB);

            telemetry.addData("alpha color", robot.pipeline.color);

            telemetry.update();
            alphaColor = robot.pipeline.color;
        }

        waitForStart();


        gyroMove.driveStraight(.5, 60, 0);
        gyroMove.turnToHeading(0.5, -90);

        //if arm can reach without the robot moving then delete this
        for (int repeats = 0; repeats < 5; repeats++)
        {
            gyroMove.driveStraight(.5, 23, -90);
            //add arm movements here
            gyroMove.driveStraight(.5, -39, -90);
        }
        //if arm can reach without the robot moving then delete this


        if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Green) {
            gyroMove.driveStraight(.5, 23, 0);
        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Red){
            gyroMove.driveStraight(.5, 23, 0);
            gyroMove.turnToHeading(.5, -90);
            gyroMove.driveStraight(.5, 23, 0);
        }
        else if (alphaColor == RobotHardware.PowerPlayPipeline.MarkerPosition.Blue){
            gyroMove.driveStraight(.5, 23, 0);
            gyroMove.turnToHeading(.5, 90);
            gyroMove.driveStraight(.5, 23, 0);
        }

        //robot.allpower(0);

    }
}

package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.EncoderDriveHeading;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotatePrecise;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RotateToHeading;

@Autonomous (name="BlueWall")
// hello
public class AutonomousTestBlueWall extends LinearOpMode
{
    //@Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this, 200, 165);
        EncoderDrive encoderDrive = new EncoderDrive(robot);
        EncoderDriveHeading encoderDriveHeading = new EncoderDriveHeading(robot);
        RotatePrecise rotatePrecise =  new RotatePrecise(robot);
        RotateToHeading rotateToHeading = new RotateToHeading(robot, rotatePrecise);
       // EasyOpenCVExample RingDetection = new EasyOpenCVExample();

        //robot.pipeline.getAnalysis();


        //int location = robot.pipeline.getAnalysis();
        int path = 0;

        while (!opModeIsActive())
        {
            if (robot.pipeline.position == robot.pipeline.position.FOUR)
            {
                path = 2; // 4 rings
            }
            if (robot.pipeline.position == robot.pipeline.position.ONE)
            {
                path = 1; // 1 ring
            }
            if (robot.pipeline.position == robot.pipeline.position.NONE)
            {
                path = 0; // no rings
            }
            telemetry.addData("path value = ", path);
            telemetry.update();
        }

        waitForStart();

        //telemetry.addData("path value = ", path);
        System.out.println("path value = " + path);

        final double WHITELINE_DISTANCE = 68; //72
        final double BOXLENGTH = 27; //22.75
        final double DISTANCETO_BOXB = 7; //9
        final double EXTRALENGTH = 9;

        if (path==0) // Square A, 0 rings
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE, 0, 10, true); //drives to middle of square A
            sleep(2000); // drop wobble goal
            //should already be on line
        }

        if (path==1) // Square B, 1 ring
        {
            encoderDriveHeading.StartAction(0.8, WHITELINE_DISTANCE+BOXLENGTH, 0, 10, true); //drives next to middle of square B
            // use box length to get to middle of next box from middle of first box
            rotateToHeading.DoIt(90);
            //rotatePrecise.RotatePrecise(-90, 2, 0.6, 0.3, 2); //turn to face square B
            encoderDriveHeading.StartAction(0.6, DISTANCETO_BOXB, -90, 10, true); // driving closer to the square, may not be needed
            sleep(2000); // drop wobble goal
            encoderDriveHeading.StartAction(-0.6, -DISTANCETO_BOXB, -90, 10, true); //back up to original position
            rotateToHeading.DoIt(0);
            //rotatePrecise.RotatePrecise(0, 2, 0.6, 0.3, 2); //turn back to original orientation
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH, 0, 10, true); // drive to line
        }

        if (path==2) // Square C, 4 rings
        {
            encoderDriveHeading.StartAction(0.6, WHITELINE_DISTANCE+BOXLENGTH+EXTRALENGTH, 0, 10, true); // drive to middle of square C
            // use box length to get to middle of next box from middle of first box
            sleep(2000); // drop wobble goal
            encoderDriveHeading.StartAction(0.8, -BOXLENGTH-EXTRALENGTH, 0, 10, true); //drive backwards to line
        }
    }
}

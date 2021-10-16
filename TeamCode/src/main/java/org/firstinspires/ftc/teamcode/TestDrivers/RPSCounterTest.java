package org.firstinspires.ftc.teamcode.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.EncoderFourWheelDriveHeading;
import org.firstinspires.ftc.teamcode.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Drivers.RotatePreciseFourWheelDrive;
import org.firstinspires.ftc.teamcode.Drivers.RotateToHeadingFourWheelDrive;

@Autonomous (name = "RPSTest")

public class RPSCounterTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.LEFT);
        RotatePreciseFourWheelDrive rotatePrecise =  new RotatePreciseFourWheelDrive(robot);
        RotateToHeadingFourWheelDrive rotateToHeading = new RotateToHeadingFourWheelDrive(robot, rotatePrecise);
        EncoderFourWheelDriveHeading encoderDriveHeading = new EncoderFourWheelDriveHeading(robot);

        robot.nucketyServo.setPosition(robot.nucketyUp);
        robot.sweepyServo.setPosition(robot.sweepyOut);

        telemetry.addLine("Everything set");

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        robot.frontshot.setPower(0.405);//39
        robot.backshot.setPower(0.405);

       // robot.frontshot.setPower(0.59);//63
       // robot.backshot.setPower(0.59);


        //encoderDriveHeading.StartAction(0.7, 57, 0, 5, true);

        sleep(2000);
        //robot.RPSCounter(1400);

//        for (int i = 0; i < 3; i++)
//        {
        //angled case
        /*
        double changeHeading = 6;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440
        changeHeading+=4.5;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440
        changeHeading+=6;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440

         */

     //   robot.ThreeRingLaunch(1500, 3);
        robot.ThreeRingLaunch(1390, 3);
        sleep(500);

/*
        //straight on case
        double changeHeading = 0;
        //rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440
        changeHeading+=5;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440
        changeHeading-=9;
        rotateToHeading.DoItSpecify(changeHeading, .65, 0.41, 0.1, 5);
        robot.ThreeRingLaunch(1300, 1); //1440
*/
        //changeHeading+=3;
            //changeHeading+=3;
//        }
        sleep(500);
        rotateToHeading.DoIt(0);
    }
}

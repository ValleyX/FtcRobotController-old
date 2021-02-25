package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@Autonomous (name = "RPSTest")

public class RPSCounterTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.LEFT);

        robot.nucketyServo.setPosition(robot.nucketyUp);
        robot.sweepyServo.setPosition(robot.sweepyOut);

        telemetry.addLine("Everything set");

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        robot.frontshot.setPower(0.62);
        robot.backshot.setPower(0.62);

        //robot.RPSCounter(1400);
        robot.ThreeRingLaunch(1440); //1450
        //robot.PowershotRingLaunch(1440);
    }
}

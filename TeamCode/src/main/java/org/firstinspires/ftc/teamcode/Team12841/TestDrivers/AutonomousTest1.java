package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

@Autonomous(name="AutonomousTest")
public class AutonomousTest1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoder = new EncoderDrive(robot);

        waitForStart();
        final double fullturn = 3.14 * 18; //18 inches
        final double halfturn = 3.14 * 9; // 9 inches

        encoder.StartAction(1,fullturn,-fullturn,10,true);
        sleep(2000);
        encoder.StartAction(1,halfturn,-halfturn,10, true);


    }
}

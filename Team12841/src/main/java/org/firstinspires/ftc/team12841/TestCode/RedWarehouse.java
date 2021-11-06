package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

@Autonomous(name = "red warehouse")

public class RedWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();

        //move to warehouse
        encoderDrive.StartAction(0.75, 20, 20, 5, true);
        encoderDrive.StartAction(1, -18, 18, 5, true);
        encoderDrive.StartAction(1, 55, 55, 5, true);


    }
}

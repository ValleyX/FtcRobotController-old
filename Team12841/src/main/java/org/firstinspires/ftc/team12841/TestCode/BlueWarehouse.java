package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

@Autonomous(name = "blu warehouse")

public class BlueWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this,100,100, RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();

        //move to warehouse
        encoderDrive.StartAction(0.75, 15, 15, 5, true);
        encoderDrive.StartAction(1, 17, -17, 5, true);
        encoderDrive.StartAction(1, 40, 40, 5, true);
// might need to be changed with final robot

    }
}

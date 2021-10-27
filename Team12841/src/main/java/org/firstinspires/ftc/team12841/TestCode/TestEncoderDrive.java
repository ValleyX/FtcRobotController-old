package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import java.nio.file.Watchable;

@Autonomous(name = "encoderTest1")
public class TestEncoderDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();

        encoderDrive.StartAction(0.75,16,16, .5, true);
        sleep(500);
        encoderDrive.StartAction(0.75,-12,12, .5, true);
    }
}

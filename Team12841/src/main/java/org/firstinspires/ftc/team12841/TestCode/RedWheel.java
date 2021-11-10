package org.firstinspires.ftc.team12841.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import java.nio.file.Watchable;

@Autonomous(name = "red wheel")

public class RedWheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();
//this all needs reversing:
        //move to wheel
        encoderDrive.StartAction(0.5, 5, 5, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, -19, 19, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, -22, -22, 5, true);
        sleep(200);
        encoderDrive.StartAction(.5,14,-14,5,true);

        //Below is the turntable spinning motor
        //robotHardware.WheelMotor.setPower(0.2);



        encoderDrive.StartAction(.5, 8.25, -8.25, 5, true);
        encoderDrive.StartAction(.5, 21, 21, 5, true);
        // might change with final bot
    }
}


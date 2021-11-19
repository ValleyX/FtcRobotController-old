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
        RobotHardware robotHardware = new RobotHardware(hardwareMap, this, 100, 100, RobotHardware.cameraSelection.LEFT);
        EncoderDrive encoderDrive = new EncoderDrive((robotHardware));

        waitForStart();
//this all needs reversing:
        //move to wheel
        encoderDrive.StartAction(0.5, -5, -5, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, -19, 19, 5, true);
        sleep(200);
        encoderDrive.StartAction(0.5, 20, 20, 5, true);
        sleep(200);
        encoderDrive.StartAction(.5,17,-17,5,true);

        //Below is the turntable spinning motor
        robotHardware.WheelMotor.setPower(-1);
        sleep(3000);
        robotHardware.WheelMotor.setPower(0);



        encoderDrive.StartAction(.5, 4, -4, 5, true);
        encoderDrive.StartAction(.5, -18, -18, 5, true);
        // might change with final bot
    }
}


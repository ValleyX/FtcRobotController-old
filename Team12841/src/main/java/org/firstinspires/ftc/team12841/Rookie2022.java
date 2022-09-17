package org.firstinspires.ftc.team12841;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestDriverRookie")
public class Rookie2022 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class,"rightRear");

      waitForStart();

        while (true) {
           boolean buttona;
           double gamepadlefty;
           double gamepadrighty;
           gamepadlefty = gamepad1.left_stick_y;
           gamepadrighty = gamepad1.right_stick_y;
           leftFront.setPower(gamepadlefty);
           rightFront.setPower(gamepadrighty);
           leftRear.setPower(gamepadlefty);
            rightRear.setPower(gamepadrighty);
        }
    }
}

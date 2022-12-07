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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

      waitForStart();

        while (opModeIsActive()) {
           boolean buttona;
           double gamepadlefty;
           double gamepadrighty;
           gamepadlefty = gamepad1.left_stick_y;
           gamepadrighty = gamepad1.right_stick_y;
           leftFront.setPower(gamepadlefty);
           rightFront.setPower(gamepadrighty);
           leftRear.setPower(gamepadlefty);
           rightRear.setPower(gamepadrighty);
            telemetry.addData("left stick y", gamepadlefty);
            telemetry.addData("right stick y", gamepadrighty);
            telemetry.update();

            if ((gamepadlefty > 0.1) && (gamepadrighty > 0.1)) {
                System.out.println("ValleyX left stick " + gamepadlefty);
                System.out.println("ValleyX right stick " + gamepadrighty);
            }
            else if (true){
                System.out.println("ValleyX EmilySpecialCode");
            }
            else if (false)
            {

            }



        }
    }
}

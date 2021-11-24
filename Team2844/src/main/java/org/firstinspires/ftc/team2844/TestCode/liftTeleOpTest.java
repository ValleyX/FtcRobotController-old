package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp (name = "lift test")

public class liftTeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftmotor;

        liftmotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        waitForStart();

        while (opModeIsActive()){

            liftmotor.setPower(-gamepad1.left_stick_y);

        }



    }
}

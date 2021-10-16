package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestMechaWheels")
public class TestTheMechaWheels extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            double strafeR = gamepad1.right_trigger;
            double strafeL = gamepad1.left_trigger;
            double lefty = gamepad1.left_stick_y;
            double righty = gamepad1.right_stick_y;
            //moving
            if (gamepad1.right_trigger > 0){
                leftFront.setPower(-strafeR);
                leftBack.setPower(strafeR);
                rightBack.setPower(-strafeR);
                rightFront.setPower(strafeR);
            }
            else if (gamepad1.left_trigger > 0.0) {  //is pressed

                leftFront.setPower(strafeL);
                leftBack.setPower(-strafeL);
                rightBack.setPower(strafeL);
                rightFront.setPower(-strafeL);
            }
            else {

                leftFront.setPower(lefty);
                leftBack.setPower(lefty);
                rightFront.setPower(righty);
                rightBack.setPower(righty);
            }

//telemetry for phone for driving
            telemetry.addData("leftstick y", lefty);
            telemetry.addData("rightstick y", righty);
            telemetry.addData("strafeR y", strafeR);
            telemetry.addData("strafeL y", strafeL);
            telemetry.update();

        }
    }
}


package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestMechaWheelsDriver")
public class TestTheMechaWheels extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //the same name for the motor on the phone
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");

        //motors have to be reversed because of the gears or the orientation of the motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {
            //using different buttons on controller
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


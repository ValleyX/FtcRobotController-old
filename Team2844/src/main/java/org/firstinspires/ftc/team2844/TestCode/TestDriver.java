package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Disabled

@TeleOp (name="TestDriver")
public class TestDriver extends LinearOpMode {
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
            /*
            double lefty = gamepad1.left_stick_y;
            leftFront.setPower(lefty);
            leftBack.setPower(lefty);

            double righty = gamepad1.right_stick_y;
            rightFront.setPower(righty);
            rightBack.setPower(righty);
*/
            if (gamepad1.left_bumper){
                rightFront.setPower(1);
                rightBack.setPower(-1);
                leftFront.setPower(-1);
                leftBack.setPower(1);
            }


        }

    }
}

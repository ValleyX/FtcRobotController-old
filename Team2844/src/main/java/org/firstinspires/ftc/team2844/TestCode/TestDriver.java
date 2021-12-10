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
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0,0, RobotHardware.cameraSelection.LEFT);

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
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(-1);
                robot.leftFront.setPower(-1);
                robot.leftBack.setPower(1);
            }



            if (gamepad1.right_bumper) {
                robot.duckySpinner.setPower(1);
            }

            double power = 0 ;
            if (gamepad1.dpad_up) {
                double newpower= power += 0.1;
                robot.duckySpinner.setPower(newpower);
                power = newpower;

            }

            if (gamepad1.dpad_down) {
                double newpower= power += 0.1;
                robot.duckySpinner.setPower(power);
                power = newpower;

            }


        }

    }
}

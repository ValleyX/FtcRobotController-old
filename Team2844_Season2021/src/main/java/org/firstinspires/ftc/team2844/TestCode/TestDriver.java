package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@TeleOp (name="TestDriver")
public class TestDriver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0,0, RobotHardware.cameraSelection.DOWN);

        double power = 0;
        boolean pressedup = false;
        boolean presseddown = false;
        double strafeR = gamepad1.right_trigger;
        double strafeL = gamepad1.left_trigger;
        double lefty = -gamepad1.left_stick_y; // try to change to - so that we can use
        double righty = -gamepad1.right_stick_y; // the actual robothardware
        double lift = -gamepad2.right_stick_y;

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

            if (gamepad1.right_trigger > 0){
                robot.leftFront.setPower(-strafeR);
                robot.leftBack.setPower(strafeR);
                robot.rightBack.setPower(-strafeR);
                robot.rightFront.setPower(strafeR);
            }
            else if (gamepad1.left_trigger > 0.0) {  //is pressed

                robot.leftFront.setPower(strafeL);
                robot.leftBack.setPower(-strafeL);
                robot.rightBack.setPower(strafeL);
                robot.rightFront.setPower(-strafeL);
            }
            else {

                robot.leftFront.setPower(lefty);
                robot.leftBack.setPower(lefty);
                robot.rightFront.setPower(righty);
                robot.rightBack.setPower(righty);
            }


            if (gamepad1.dpad_left == true  ) {
                robot.duckySpinner.setPower(0.94);
            }

            else {
                robot.duckySpinner.setPower(0);
            }

            if( gamepad1.dpad_right ){
                robot.duckySpinner.setPower(0);

            }


            if (gamepad1.dpad_up && pressedup == false) {
                power += 0.01;
                robot.duckySpinner.setPower(power);
                pressedup = true;

            }
            else if (!gamepad1.dpad_up) {
                pressedup= false;
            }

            if (gamepad1.dpad_down && presseddown == false) {
                power -= 0.01;
                robot.duckySpinner.setPower(power);
                presseddown = true;
            }
            else if (!gamepad1.dpad_down) {
                presseddown= false;
            }

            if (gamepad1.right_bumper) {
                robot.duckySpinner.setPower(0);
            }

            telemetry.addData("speed of duck", robot.duckySpinner.getPower());
            telemetry.update();


        }

    }
}

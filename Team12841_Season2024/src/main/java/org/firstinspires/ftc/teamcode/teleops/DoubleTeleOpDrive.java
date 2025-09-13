package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import RobotHardwares.RobotHardware;

@TeleOp(name = "Two People")
public class DoubleTeleOpDrive extends LinearOpMode {
    RobotHardware robothardware;

    double y; // Remember, this is reversed!
    double x; // Counteract imperfect strafing
    double rx;

    double frontLeftActualPower;
    double backLeftActualPower;
    double frontRightActualPower;
    double backRightActualPower;

    boolean minorMode = false;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this);

        waitForStart();
        if (isStopRequested()) return;
        timer.reset();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            //Bot heading is in radians not degrees!!!!!!!!!!!!
            double botHeading = -(double) robothardware.getHeadingRadians(); //change this to getCurrentHeading for field centric, 0 for robot centric
//            double botHeading = -(double) 0;
            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(rotY + rotX + rx) / denominator;
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            double backRightPower = -(rotY + rotX - rx) / denominator;

            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.update();

            robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);


            //Unused graveyard of code.
//            if(gamepad1.right_stick_y > 0.75){ //straight up
//
//            } else if (0.25 < gamepad1.right_stick_y && gamepad1.right_stick_x < 0){ // top left
//
//            } else if (0.25 < gamepad1.right_stick_y && gamepad1.right_stick_x > 0){ // top right
//
//            } else if (gamepad1.right_stick_y < -0.75){ //straight down
//
//            } else if (-0.25 < gamepad1.right_stick_y && gamepad1.right_stick_x < 0){ //bottom left
//
//            } else if (-0.25 < gamepad1.right_stick_y && gamepad1.right_stick_x > 0){ //bottom right
//
//            } else if (gamepad1.right_stick_x < -0.75){ // straight left
//
//            } else if (gamepad1.right_stick_x > 0.75){ // straight right
//
//            }

//            if (Math.abs(gamepad1.left_stick_x) > .01 || Math.abs(gamepad1.left_stick_y) > .01 || Math.abs(gamepad1.right_stick_x) > .01) {
//
//                 if (Math.abs(gamepad1.left_stick_x) > .01) {
//                    robothardware.strafe(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
//                }
//
//                 if (Math.abs(gamepad1.left_stick_y) > .5) {
//                    robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
//                }
//
//                if (Math.abs(gamepad1.right_stick_x) > .5) {
//                    robothardware.turnBot(gamepad1.right_stick_x);
//                }
//            }
//            else {
//                robothardware.allPower(0, 0, 0, 0);
//            }

        }
    }
}

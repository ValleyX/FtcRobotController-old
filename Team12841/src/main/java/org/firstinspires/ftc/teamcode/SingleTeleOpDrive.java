package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@TeleOp(name = "One Person")
public class SingleTeleOpDrive extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.BlueL);
        lifthardware = new LiftHardware(robothardware, this);

        robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double y; // Remember, this is reversed!
        double x; // Counteract imperfect strafing
        double rx;

        //elbow variables
        int elbowPosition;
        boolean elbowDpadPressed = false;
        final int STRAIGHT_UP = 3438;
        final int BOARD_PLACEMENT = 3879;
        final int MAX_ELBOW = 4670;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            elbowPosition = robothardware.elbowMotor.getCurrentPosition();
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.2;
            rx = gamepad1.right_stick_x;

            //all of the math for field centric
            double botHeading = 0; //change this to getCurrentHeading for field centric, 0 for robot centric
            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);//more math
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;//even more math


            //makes the motors actually move
            robothardware.leftFrontDrive.setPower(frontLeftPower);
            robothardware.leftBackDrive.setPower(backLeftPower);
            robothardware.rightFrontDrive.setPower(frontRightPower);
            robothardware.rightBackDrive.setPower(backRightPower);

            //all the telemetry code
            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("elbow ticks", robothardware.elbowMotor.getCurrentPosition());
            telemetry.addData("elevator power", robothardware.elevatorMotor.getPower());
            telemetry.addData("elevator ticks", robothardware.elevatorMotor.getCurrentPosition());
            telemetry.addData("bucket position", robothardware.bucket.getPosition());
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.update();

            //all of the elevator code: assigned to up/down dpad
            if (robothardware.elbowMotor.getCurrentPosition() > 1000) {
                if (gamepad1.dpad_up) {
                    robothardware.elevatorMotor.setPower(-0.5);
                } else if (gamepad1.dpad_down) {
                    robothardware.elevatorMotor.setPower(0.5);
                } else {
                    robothardware.elevatorMotor.setPower(0);
                }
            }

            //all of the button commands for the elbow and elevator
            if (gamepad1.y)
                lifthardware.moveElbow(STRAIGHT_UP, 1);
            else if (gamepad1.b)
                lifthardware.moveElbow(BOARD_PLACEMENT, 1);
            else if (gamepad1.x)
                lifthardware.moveElbow(0, 1);
            else if (gamepad1.a)
                lifthardware.moveElevator(0, 0.6);

            //the elbow code: assigned to right/left dpad
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                if (elbowPosition >= 0 && elbowPosition <= MAX_ELBOW) {
                    robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (gamepad1.dpad_right) {
                        robothardware.elbowMotor.setPower(0.5);
                    } else if (gamepad1.dpad_left) {
                        robothardware.elbowMotor.setPower(-0.5);
                    }
                    elbowDpadPressed = true;
                }
            }

            //Safety measures to make sure the elbow doesn't move to far down or up
            else if (elbowDpadPressed) {
                if (elbowPosition < 0) {
                    elbowPosition = 0; //go to 0
                } else if (elbowPosition > MAX_ELBOW) {
                    elbowPosition = MAX_ELBOW; //go to MAX_ELBOW
                }
                robothardware.elbowMotor.setTargetPosition(elbowPosition);
                robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowDpadPressed = false;
            }

            //code for the intake
            if (gamepad1.left_trigger > 0.1) {
                robothardware.intakeMotor.setPower(gamepad1.left_trigger);
                lifthardware.openBucket();
            } else {
                robothardware.intakeMotor.setPower(-gamepad1.right_trigger);
                lifthardware.closeBucket();
            }
        }
    }
}
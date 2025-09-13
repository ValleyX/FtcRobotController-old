package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.RobotHardware;
import RobotHardwares.LiftHardware;

@TeleOp(name = "Tanner - TeleOp")
public class TannerTeleOp extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;
    LinearOpMode opMode_;

    double y; // Remember, this is reversed!
    double x; // Counteract imperfect strafing
    double rx;

    double minorPower = 5;
    double slideSpeed = 1.0;

    // Toggles
    boolean robotCentric = false;
    boolean pressingRobo = false;
    boolean rainbowMode = false;
    boolean pressingRainbow = false;
    boolean yPress = false;
    boolean pressingY = false;
    boolean overRide = false;
    boolean pressingX = false;

    // Holding buttons
    boolean minorMode = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this);
        lifthardware = new LiftHardware(robothardware, this);
        opMode_ = this;

        robothardware.imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;
        timer.reset();
        while (opModeIsActive()) {

            // Toggle robot centric mode
            if (gamepad1.left_trigger > 0.5) {
                if (!pressingRobo) {
                    robotCentric = !robotCentric;
                    pressingRobo = true;
                }
            } else {
                pressingRobo = false;
            }

            // Toggle rainbow mode
            if (gamepad1.y) {
                if (!pressingRainbow) {
                    rainbowMode = !rainbowMode;
                    pressingRainbow = true;
                }
            } else {
                pressingRainbow = false;
            }

            // Minor mode based on triggers or slide height
            if (gamepad1.right_trigger > 0.1) {
                minorMode = true;
                minorPower = 3;
            } else if (lifthardware.getYSlidesHeight() >= 17) {
                minorMode = true;
                minorPower = 2;
            } else {
                minorMode = false;
            }

            // Driving controls
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (minorMode) {
                y /= minorPower;
                x /= minorPower;
                rx /= minorPower;
            }

            // Calculate bot heading RADIANS
            double botHeading = robotCentric ? 0.0 : -robothardware.getHeadingRadians();

            // Field centric calculations
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(rotY + rotX + rx) / denominator;
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            double backRightPower = -(rotY + rotX - rx) / denominator;

            // Telemetry
            telemetry.addData("botHeading", botHeading);
            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.addData("Minor Mode", minorMode);
            telemetry.addData("Servo Pos.", lifthardware.getClawPos());
            telemetry.addData("Wrist Position", robothardware.clawWrist.getPosition());
            telemetry.addData("Extend Power", robothardware.extendMotor.getPower());
            telemetry.addData("Extend throttle power", gamepad2.left_stick_y);
            telemetry.addData("Y Slides Motor Left Encoder", robothardware.ySlidesMotorLeft.getCurrentPosition());
            telemetry.addData("Y Slides Motor Right Encoder", robothardware.ySlidesMotorRight.getCurrentPosition());
            telemetry.addData("Y Slides Height in Inches", lifthardware.getYSlidesHeight());
            telemetry.addData("X Slides Length Encoder", robothardware.pivotMotor.getCurrentPosition());
            telemetry.update();

            // Set motor powers
            robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
            robothardware.extendMotor.setPower(-gamepad2.left_stick_y * slideSpeed);

            // Reset IMU yaw
            if (gamepad1.a) {
                robothardware.imu.resetYaw();
            }

            // Slide speed control
            if (gamepad2.dpad_up) {
                slideSpeed = 1.0;
            } else if (gamepad2.dpad_down) {
                slideSpeed = 0.5;
            }

            // Pivot control
            lifthardware.powerPivot(gamepad2.right_stick_y * slideSpeed);
            double wristPos = robothardware.clawWrist.getPosition();
            if (gamepad2.right_trigger > 0.1) {
                robothardware.clawWrist.setPosition(wristPos + 0.035);
            } else if (gamepad2.right_bumper) {
                robothardware.clawWrist.setPosition(wristPos - 0.035);
            }

            // Manipulator encoder resets
            if (gamepad2.back) {
                robothardware.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robothardware.ySlidesMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robothardware.ySlidesMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Claw control
            if (gamepad2.b) {
                lifthardware.closeClaw();
            } else if (gamepad2.a) {
                lifthardware.openClaw();
            }

            // Y slides control
            if (gamepad2.left_trigger > 0.1) {
                lifthardware.powerYSlides(-slideSpeed, overRide);
                yPress = false;
            } else if (gamepad2.left_bumper) {
                lifthardware.powerYSlides(slideSpeed, overRide);
                yPress = false;
            } else if (yPress) {
                lifthardware.moveYSlides(44, slideSpeed);
            } else {
                lifthardware.stopYSlides();
            }
        }
    }
}

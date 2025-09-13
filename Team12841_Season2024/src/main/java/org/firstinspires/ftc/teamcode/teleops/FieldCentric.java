package org.firstinspires.ftc.teamcode.teleops;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import RobotHardwares.RobotHardware;
import RobotHardwares.LiftHardware;


@TeleOp(name = "Field Centric")
@Disabled
public class FieldCentric extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;
    LinearOpMode opMode_;

    double y; // Remember, this is reversed!
    double x; // Counteract imperfect strafing
    double rx;

    double frontLeftActualPower;
    double backLeftActualPower;
    double frontRightActualPower;
    double backRightActualPower;

    double slideSpeed = 1.0;

    boolean robotCentric = false;
    boolean rainbowMode = false;
    boolean minorMode = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this);
        lifthardware = new LiftHardware(robothardware, this);
        opMode_ = this;


        waitForStart();
        if (isStopRequested()) return;
        timer.reset();
        while (opModeIsActive()) {

            //if the driver's left trigger is pulled turn off feild centric
            if (gamepad1.left_trigger > 0.5) {
                robotCentric = !robotCentric;
                opMode_.sleep(100); //the wait is so the driver can remove his finger without the boolean switching back
            }

            if (gamepad1.y) {
                rainbowMode = !rainbowMode;
                opMode_.sleep(100); //the wait is so the driver can remove his finger without the boolean switching back
            }


            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
//            rx = 0;
            //Bot heading is in radians not degrees!!!!!!!!!!!!
            double botHeading;
            if (robotCentric) {
                botHeading = 0.0;
            } else {
                botHeading = -(double) robothardware.getHeadingRadians(); //change this to getCurrentHeading for field centric, 0 for robot centric
            }

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

            if (gamepad1.right_trigger > 0.0) { //minor mode when held down
                minorMode = true;
            } else { //if not holding trigger down just run like normal
                minorMode = false;
            }


            robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);


            if (gamepad1.a) { //if the driver presses a the imu is reset
                robothardware.imu.resetYaw();
            }
        }
    }
}

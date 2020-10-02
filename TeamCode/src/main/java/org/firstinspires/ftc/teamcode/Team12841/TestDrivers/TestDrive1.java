package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

@TeleOp (name="DriverTest")
public class TestDrive1 extends LinearOpMode

{
    public void runOpMode ()
    {
        RobotHardware robot = new RobotHardware(hardwareMap, this);

        double left;
        double right;

        waitForStart();

        while (opModeIsActive()) {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            if (right >= .5)
                right = .5;
            if (left >= .5)
                left =.5;

            if (right <= -0.5)
                right = -0.5;
            if (left <= -0.5)
                left =-0.5;

            if ((left == 0) && (right == 0)) {
                telemetry.addLine("still");
            }
            else {
                telemetry.addLine("moving");
            }





            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            telemetry.addData("LeftStick = ", left);
            telemetry.addData("RightStick = ", right);
            telemetry.update();
        }
    }
}

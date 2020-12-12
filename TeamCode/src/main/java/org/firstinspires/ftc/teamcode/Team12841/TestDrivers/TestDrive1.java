package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware4motors;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name="DriverTest")
public class TestDrive1 extends LinearOpMode

{
    public void runOpMode ()
    {
        RobotHardware4motors robot = new RobotHardware4motors(hardwareMap, this);

        double left;
        double right;

        waitForStart();

        while (opModeIsActive()) {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;


            if ((left == 0) && (right == 0)) {
                telemetry.addLine("still");
            }
            else {
                telemetry.addLine("moving");
            }


            robot.leftDrivefront.setPower(left);
            robot.rightDrivefront.setPower(right);
            //robot.leftDriveback.setPower(left);
            //robot.rightDriveback.setPower(right);

            telemetry.addData("LeftStick = ", left);
            telemetry.addData("RightStick = ", right);
            telemetry.update();
        }
    }
}


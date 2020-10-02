package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp (name="DriverTest")

public class TestDriverMode extends LinearOpMode
{
    @Override
    public void runOpMode ()
    {
         RobotHardware robot = new RobotHardware(hardwareMap, this);
         System.out.println("ValleyX: In Innit");
         waitForStart();

         while (opModeIsActive()) {
             double left;
             double right;

             left = -gamepad1.left_stick_y;
             right = -gamepad1.right_stick_y;

             robot.leftDrive.setPower(left);
             robot.rightDrive.setPower(right);

             telemetry.addData("LeftStickY = ",left);
             telemetry.addData("RightStickY = ", right);
             telemetry.update();
         }



    }

}

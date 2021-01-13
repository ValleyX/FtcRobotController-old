package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp (name="DriverTest")

public class TestDriverMode extends LinearOpMode
{
    @Override
    public void runOpMode ()
    {
         MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.RIGHT);
         System.out.println("ValleyX: In Innit");
         waitForStart();

         while (opModeIsActive()) {
             double left;
             double right;
             double rtrigger;
             double ltrigger;

             // Gamepad 1
             left = -gamepad1.left_stick_y;
             right = -gamepad1.right_stick_y;

             robot.leftFrontDrive.setPower(left);
             robot.leftBackDrive.setPower(left);
             robot.rightFrontDrive.setPower(right);
             robot.rightBackDrive.setPower(right);

             rtrigger = gamepad1.right_trigger;
             ltrigger = gamepad1.left_trigger;

             // set intake motors to l and r triggers

             telemetry.addData("LeftStickY = ",left);
             telemetry.addData("RightStickY = ", right);
             telemetry.update();

             // Gamepad 2
             // shooter (motors)
             // box (servo), presets for intake and loading
             // box ring pushing servo (incorporate into box preset buttons??)
             // wobble goal arm (servo)



             // lights
         }



    }

}

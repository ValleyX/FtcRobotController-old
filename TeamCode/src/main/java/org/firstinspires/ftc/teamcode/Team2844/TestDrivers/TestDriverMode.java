package org.firstinspires.ftc.teamcode.Team2844.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team2844.Drivers.MandoRobotHardware;
import org.firstinspires.ftc.teamcode.Team2844.Drivers.RobotHardware;

@TeleOp (name="TestDriverMode")

public class TestDriverMode extends LinearOpMode
{
    @Override
    public void runOpMode ()
    {
         MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.RIGHT);
         System.out.println("ValleyX: In Innit");
         waitForStart();

         boolean bDpadup = false;
         boolean bDpaddown = false;
         double currentSpeed = 0.5;


        while (opModeIsActive())
         {
             double left;
             double right;
             double rtrigger;
             double ltrigger;

             // Gamepad 1
             // driving
             left = -gamepad1.left_stick_y;
             right = -gamepad1.right_stick_y;

             robot.leftFrontDrive.setPower(left);
             robot.leftBackDrive.setPower(left);
             robot.rightFrontDrive.setPower(right);
             robot.rightBackDrive.setPower(right);

             // intake
             rtrigger = gamepad1.right_trigger;
             ltrigger = gamepad1.left_trigger;
             // set intake motors to l and r triggers

             // wobble goal arm
             if (gamepad1.x)
             {
                 robot.wobbleServo.setPosition(1.0); //down
             }
             if (gamepad1.y)
             {
                 robot.wobbleServo.setPosition(0.0); //up
             }
             if (gamepad1.a)
             {
                 robot.clasper.setPosition(0.5);
             }
             if (gamepad1.b)
             {
                 robot.clasper.setPosition(0.0);
             }
             telemetry.addData("LeftStickY = ",left);
             telemetry.addData("RightStickY = ", right);

             // Gamepad 2
             // shooter (motors)
             if (gamepad2.dpad_up)
             {
                 if (!bDpadup) {
                     bDpadup = true;
                     currentSpeed = currentSpeed - 0.05;
                 }
                 robot.backshot.setPower(currentSpeed);
                 robot.frontshot.setPower(currentSpeed);
             }
             else
             {
                 bDpadup = false;
             }
             if (gamepad2.dpad_down)
             {
                 if (!bDpaddown)
                 {
                     bDpaddown = true;
                     currentSpeed = currentSpeed + 0.05;
                 }
                 robot.backshot.setPower(currentSpeed);
                 robot.frontshot.setPower(currentSpeed);
             }
             else
             {
                 bDpaddown = false;
             }
             telemetry.addData("current speed: ", currentSpeed);
             telemetry.update();

             // box (servo), presets for intake and loading
             if (gamepad2.a)
             {
                 robot.nucketyServo.setPosition(0.5); // down
             }
             if (gamepad2.b)
             {
                 robot.nucketyServo.setPosition(0.05); //up
             }
             // box ring pushing servo (incorporate into box preset buttons??)
             if (gamepad2.x)
             {
                 robot.sweepyServo.setPosition(0.75); //out
             }
             if (gamepad2.y)
             {
                 robot.sweepyServo.setPosition(0.5); //push
             }

             // lights
         }
    }
}

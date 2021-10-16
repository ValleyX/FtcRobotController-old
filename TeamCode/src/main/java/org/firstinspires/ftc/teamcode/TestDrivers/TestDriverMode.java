package org.firstinspires.ftc.teamcode.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.MandoRobotHardware;

@TeleOp (name="TestDriverMode")

public class TestDriverMode extends LinearOpMode
{
    private ElapsedTime runtime_ = new ElapsedTime();
    MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.RIGHT);

    public void driveWhileWaiting(int waitTime)
    {
        while (waitTime < runtime_.milliseconds())
        {
            sleep(200);

            double left = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;
            robot.leftFrontDrive.setPower(left);
            robot.leftBackDrive.setPower(left);
            robot.rightFrontDrive.setPower(right);
            robot.rightBackDrive.setPower(right);
        }
    }

    @Override
    public void runOpMode()
    {
         MandoRobotHardware robot = new MandoRobotHardware(this, 0, 0, MandoRobotHardware.cameraSelection.RIGHT);
         System.out.println("ValleyX: In Innit");
         waitForStart();

         double idealSpeed = 0.4; //0.6
         double powerShot = 0.35;
         boolean bucketDown = true;

        while (opModeIsActive())
         {
             double left;
             double right;
             double rtrigger;
             double ltrigger;

             /***************************************Gamepad 1 - Driver*************************************/
             // driving
             left = -gamepad1.left_stick_y;
             right = -gamepad1.right_stick_y;
             robot.leftFrontDrive.setPower(left);
             robot.leftBackDrive.setPower(left);
             robot.rightFrontDrive.setPower(right);
             robot.rightBackDrive.setPower(right);

             // wobble goal arm
             if (gamepad1.x) {
                 robot.wobbleServo.setPosition(robot.wobbleDown); //down
             }
             if (gamepad1.y) {
                 robot.clasper.setPosition(robot.clasperClosed); // make sure clasper is closed before moving the wobble goal arm
                 driveWhileWaiting(500);
                 robot.wobbleServo.setPosition(robot.wobbleUp); //up
             }
             if (gamepad1.a) {
                 robot.clasper.setPosition(robot.clasperOpen);
             }
             if (gamepad1.b) {
                 robot.clasper.setPosition(robot.clasperClosed);
             }

             telemetry.addData("LeftStickY = ",left);
             telemetry.addData("RightStickY = ", right);

             /***************************************Gamepad 2 - Manipulator*************************************/
             // shooter (motors)
             if (gamepad2.left_bumper) // turn on shooter wheels
             {
                 robot.backshot.setPower(idealSpeed);
                 robot.frontshot.setPower(idealSpeed);
             }
             if (gamepad2.right_bumper) // turn off shooter wheels
             {
                 robot.backshot.setPower(0.0);
                 robot.frontshot.setPower(0.0);
             }

             //bucket
             if (gamepad2.dpad_up) // bucket up, and make sure sweepy out
             {
                 robot.nucketyServo.setPosition(robot.nucketyUp);
                 robot.sweepyServo.setPosition(robot.sweepyOut);

                 robot.backshot.setPower(idealSpeed);
                 robot.frontshot.setPower(idealSpeed);

                 bucketDown = false;
             }
             if (gamepad2.dpad_down) // bucket down, and make sure sweepy out
             {
                 robot.nucketyServo.setPosition(robot.nucketyDown);
                 robot.sweepyServo.setPosition(robot.sweepyOut);

                 robot.backshot.setPower(0.0);
                 robot.frontshot.setPower(0.0);
                 bucketDown = true;
             }

             //sweepy push and back out to launch rings
             if (gamepad2.a && bucketDown == false)
             {
                 robot.sweepyServo.setPosition(robot.sweepyPush);
                 driveWhileWaiting(500);
                 robot.sweepyServo.setPosition(robot.sweepyOut);
             }

             if (gamepad2.dpad_right)
             {
                 robot.backshot.setPower(powerShot);
                 robot.frontshot.setPower(powerShot);
             }

             // intake
             rtrigger = gamepad2.right_trigger;
             ltrigger = -gamepad2.left_trigger;
             // set intake motors to l and r triggers
             robot.intake.setPower(rtrigger);
             robot.intake.setPower(ltrigger);
             // make sure bucket is down and sweepy is out while intaking
             if (robot.intake.getPower() != 0)
             {
                 robot.nucketyServo.setPosition(robot.nucketyDown);
                 robot.sweepyServo.setPosition(robot.sweepyOut);
                 robot.backshot.setPower(0.0);
                 robot.frontshot.setPower(0.0);
             }
         }
    }
}

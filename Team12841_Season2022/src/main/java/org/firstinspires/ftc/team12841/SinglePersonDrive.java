package org.firstinspires.ftc.team12841;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.team12841.drivers.LiftDriver_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.RobotHardware;


@TeleOp (name = "SinglePersonDrive(ForPractice)")
public class SinglePersonDrive extends LinearOpMode{

   RobotHardware robothardware;
   LiftDriver_UnderthehoodStuff liftdriver;

    @Override
    public void runOpMode () throws InterruptedException {
        robothardware = new RobotHardware(this);
        liftdriver = new LiftDriver_UnderthehoodStuff(robothardware);
        RobotGyroscope Gyro = new RobotGyroscope(robothardware);

        //Variables decloration
        double leftY;
        double rightY;
        double liftUpDown;
        int liftPosition;
        boolean stickMovement = false;
        final double liftUpSpeed = 0.2;
        final double liftDownSpeed = -0.1;
        double speed = 0.5;



        //wait...............go
        waitForStart();

        //controls
        while(opModeIsActive()) {
            liftUpDown = -gamepad1.right_stick_y;
            liftPosition = robothardware.liftMotor.getCurrentPosition();

            //El Senor pinch
            if (gamepad1.right_bumper)
            {
                robothardware.pinch.setPosition(robothardware.openPinch);
            }
            else if(gamepad1.left_bumper)
            {
                robothardware.pinch.setPosition(robothardware.closedPinch);
            }

            if (gamepad1.dpad_up)
            {
                robothardware.lFMotor.setPower(speed);
                robothardware.lBMotor.setPower(speed);
                robothardware.rFMotor.setPower(speed);
                robothardware.rBMotor.setPower(speed);
            }
            else if (gamepad1.dpad_down)
            {
                robothardware.lFMotor.setPower(-speed);
                robothardware.lBMotor.setPower(-speed);
                robothardware.rFMotor.setPower(-speed);
                robothardware.rBMotor.setPower(-speed);
            }
            else if (gamepad1.dpad_left)
            {
                robothardware.lFMotor.setPower(-speed);
                robothardware.lBMotor.setPower(-speed);
                robothardware.rFMotor.setPower(speed);
                robothardware.rBMotor.setPower(speed);
            }
            else if (gamepad1.dpad_right)
            {
                robothardware.lFMotor.setPower(speed);
                robothardware.lBMotor.setPower(speed);
                robothardware.rFMotor.setPower(-speed);
                robothardware.rBMotor.setPower(-speed);
            }
            else
            {
                robothardware.lFMotor.setPower(0);
                robothardware.lBMotor.setPower(0);
                robothardware.rFMotor.setPower(0);
                robothardware.rBMotor.setPower(0);
            }

            //short pole
            if (gamepad1.x)
            {
                liftdriver.moveInches(13,.5);
            }

            //Floor
            else if(gamepad1.a)
            {
                liftdriver.moveInches(0,.5);
            }
            //medium pole
            else if (gamepad1.y)
            {
                liftdriver.moveInches(22,.5);
            }
            //tall pole
            else if(gamepad1.b)
            {
                liftdriver.moveInches(41,.5);
            }
            else if (Math.abs(liftUpDown) > 0.2)
            {
                if(liftPosition > -5)
                {
                    robothardware.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robothardware.liftMotor.setPower((liftUpDown > 0) ? liftUpSpeed : liftDownSpeed);
                    //if boat then yes
                    stickMovement = true;
                }
            }
            else if (stickMovement == true)
            {
                robothardware.liftMotor.setTargetPosition(liftPosition);
                robothardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //gamepad1.rumbleQueue(0.5);
                stickMovement = false;
            }


            telemetry.addData("y = " , gamepad1.y);
            telemetry.addData("x = " , gamepad1.x);
            telemetry.addData("b = " , gamepad1.b);
            telemetry.addData("a = " , gamepad1.a);
            telemetry.addData("liftPosition",liftPosition);
            telemetry.addData("liftUpDown", liftPosition);
            telemetry.update();
        }
    }
}
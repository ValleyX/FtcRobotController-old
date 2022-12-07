package org.firstinspires.ftc.team12841;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team12841.drivers.LiftDriver_UnderthehoodStuff;
import org.firstinspires.ftc.team12841.drivers.RobotHardware;


@TeleOp (name = "RookieBasicDrive")
public class RookieBasicDrive extends LinearOpMode{

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
        final double creepSpeed = 0.15;
        final double turnCreepSpeed = 0.3;
        int liftPosition;
        boolean stickMovement = false;
        final double liftUpSpeed = 0.2;
        final double liftDownSpeed = -0.1;
        double speed = 0.6;



        //wait...............go
        waitForStart();

        //controls
        while(opModeIsActive()) {
            leftY = -gamepad1.left_stick_y;
            rightY = -gamepad1.right_stick_y;
            liftUpDown = -gamepad2.left_stick_y;
            liftPosition = robothardware.liftMotor.getCurrentPosition();

            //El Senor pinch
            if (gamepad2.right_bumper)
            {
                robothardware.pinch.setPosition(robothardware.openPinch);
            }
            else if(gamepad2.left_bumper)
            {
                robothardware.pinch.setPosition(robothardware.closedPinch);
            }

            if (gamepad1.dpad_up)
            {
                robothardware.lFMotor.setPower(creepSpeed);
                robothardware.lBMotor.setPower(creepSpeed);
                robothardware.rFMotor.setPower(creepSpeed);
                robothardware.rBMotor.setPower(creepSpeed);
            }
            else if (gamepad1.dpad_down)
            {
                robothardware.lFMotor.setPower(-creepSpeed);
                robothardware.lBMotor.setPower(-creepSpeed);
                robothardware.rFMotor.setPower(-creepSpeed);
                robothardware.rBMotor.setPower(-creepSpeed);
            }
            else if (gamepad1.dpad_left)
            {
                robothardware.lFMotor.setPower(-turnCreepSpeed);
                robothardware.lBMotor.setPower(-turnCreepSpeed);
                robothardware.rFMotor.setPower(turnCreepSpeed);
                robothardware.rBMotor.setPower(turnCreepSpeed);
            }
            else if (gamepad1.dpad_right)
            {
                robothardware.lFMotor.setPower(turnCreepSpeed);
                robothardware.lBMotor.setPower(turnCreepSpeed);
                robothardware.rFMotor.setPower(-turnCreepSpeed);
                robothardware.rBMotor.setPower(-turnCreepSpeed);
            }
            else if (gamepad1.left_bumper)
            {
                speed = 0.6;
            }
            else if (gamepad1.right_bumper)
            {
                speed = 1.0;
            }
            else
            {
                //tank drive
                robothardware.lFMotor.setPower(speed * leftY);
                robothardware.lBMotor.setPower(speed * leftY);
                robothardware.rFMotor.setPower(speed * rightY);
                robothardware.rBMotor.setPower(speed * rightY);
            }



            //Turning sharp and clean
            if (gamepad1.y)
            {
                Gyro.turnToHeading(1.0,0,100);
                sleep(100);
                Gyro.turnToHeading(1.0,0, 100);
            }
            if (gamepad1.x)
            {
                Gyro.turnToHeading(1.0,-90, 100);
                sleep(100);
                Gyro.turnToHeading(1.0,-90, 100);
            }
            if (gamepad1.b)
            {
                Gyro.turnToHeading(1.0,90, 100);
                sleep(100);
                Gyro.turnToHeading(1.0,90, 100);
            }
            if (gamepad1.a)
            {
                Gyro.turnToHeading(1.0,180, 100);
                sleep(100);
                Gyro.turnToHeading(1.0,180, 100);
            }


           // telemetry.update();

            //lift stuff
            //robothardware.liftMotor.setPower(liftUpDown/2);
            //short pole
            if (gamepad2.x)
            {
                liftdriver.moveInches(14,.5);
            }

            //Floor
            else if(gamepad2.a)
            {
                liftdriver.moveInches(0,.5);
            }
            //medium pole
            else if (gamepad2.y)
            {
                liftdriver.moveInches(22,.5);
            }
            //tall pole
            else if(gamepad2.b)
            {
                liftdriver.moveInches(34,.5);
            }
            else if (Math.abs(liftUpDown) > 0.2)
            {
                if(liftPosition > -5) {
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
                stickMovement = false;
            }

            telemetry.addData("left Y = ", leftY);
            telemetry.addData("right Y = " , rightY);
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
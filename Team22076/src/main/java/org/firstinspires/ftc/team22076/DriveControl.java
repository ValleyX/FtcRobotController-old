package org.firstinspires.ftc.team22076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//This code works with "LEGS" Robot controller
@TeleOp(name="JuniorHighDriveCtrl")

public class DriveControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //This creates an instance of the RobotHardware class.  This class initializes all the robot Hardware
        RobotHardware Robot = new RobotHardware(this);

        Robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftStickY;
        double rightStickX;
        boolean a_button;
        boolean y_button;
        boolean b_button;//swivels the arm up or down
        double lt_button;//lift motor for moving the arm down
        double rt_button;//lift motor control for moving the arm up
        boolean left_button;//grip
        boolean right_button;//realese
        double leftStickY2;
        boolean dpadup;
        boolean dpaddown;
        boolean dpadleftSlow;
        boolean dpadrightSlow;
        final double CREAP_VALUE = 0.3;
        boolean movement = false;
        double power_level;
        final int HIGH_POLE = 280;
        final int MED_POLE = 220;
        final int LOW_POLE = 145;

        waitForStart();

        while (opModeIsActive()) {

            leftStickY = -gamepad1.left_stick_y;
            rightStickX = -gamepad1.right_stick_x;
            b_button = gamepad1.b;
            y_button = gamepad1.y;
            lt_button = gamepad1.left_trigger;
            rt_button = gamepad1.right_trigger;
            left_button = gamepad1.left_bumper;
            right_button = gamepad1.right_bumper;
            a_button = gamepad1.a;
            dpadup = gamepad1.dpad_up;
            dpaddown = gamepad1.dpad_down;
            /*dpadupSlow = gamepad1.dpad_up;
            dpaddownSlow = gamepad1.dpad_down;
            dpadleftSlow = gamepad1.dpad_left;
            dpadrightSlow = gamepad1.dpad_right;*/


            //System.out.println("ValleyX RT " + rt_button);
            //System.out.println("ValleyX LT " + lt_button);


            if (leftStickY >= .1 || leftStickY <= -.1) {
                Robot.leftFront.setPower(leftStickY);
                Robot.leftBack.setPower(leftStickY);
                Robot.rightFront.setPower(leftStickY);
                Robot.rightBack.setPower(leftStickY);
            } else {
                Robot.rightFront.setPower(rightStickX);
                Robot.rightBack.setPower(rightStickX);
                Robot.leftFront.setPower(-rightStickX);
                Robot.leftBack.setPower(-rightStickX);
            }

            //Robot.swivelmotorRight.setPower(rt_button); //swivel Up
            // Robot.swivelmotorRight.setPower(lt_button); //swivel down
            ///
            //****This section opens and closes the claw ********//
            if (left_button) {
                Robot.gripservo.setPosition(0.33); //close
            }
            //b button = open
            if (right_button) {
                Robot.gripservo.setPosition(-0.3);  //open //BOAT
            }

            //*****This section runs the swivel motor *******///
            if (dpadup == true) {
                //Robot.liftmotor.setPower(1);
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.swivelmotorRight.setPower(.8);
                Robot.swivelmotorLeft.setPower(.8);
                //System.out.println("ValleyX dpad up" );
                movement = true;
            }
            else if (dpaddown == true) {
                //Robot.liftmotor.setPower(-1);
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.swivelmotorRight.setPower(-.25);
                Robot.swivelmotorLeft.setPower(-.25);
                //System.out.println("ValleyX dpad down" );
                movement = true;
            }
            else if (movement == true) {
                Robot.swivelmotorRight.setTargetPosition(Robot.swivelmotorRight.getCurrentPosition());
                Robot.swivelmotorLeft.setTargetPosition(Robot.swivelmotorLeft.getCurrentPosition());
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorRight.setPower(1);
                Robot.swivelmotorLeft.setPower(1);
                movement = false;
                //System.out.println("ValleyX movement true" );
            }
            else if (movement == false) {
                telemetry.addData("swivel pos Right:", Robot.swivelmotorRight.getCurrentPosition());
                telemetry.addData("swivel pos Left:", Robot.swivelmotorLeft.getCurrentPosition());
                //Robot.swivelmotorRight.setTargetPosition(Robot.swivelmotorRight.getCurrentPosition());
                //Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //System.out.println("ValleyX movement False" );
                movement = false;
            }
            else {
                //System.out.println("ValleyX movement else" );
                //Robot.liftmotor.setPower(0);
                //Robot.swivelmotorRight.setPower(0);
            }

            //Preset lift settings
            if (y_button == true) //High
            {
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorRight.setTargetPosition(HIGH_POLE);
                Robot.swivelmotorLeft.setTargetPosition(HIGH_POLE);

                Robot.swivelmotorRight.setPower(.7);
                Robot.swivelmotorLeft.setPower(.7);
                /*if (Robot.swivelmotorRight.getCurrentPosition() > HIGH_POLE) {
                    Robot.swivelmotorRight.setPower(.8);
                    Robot.swivelmotorLeft.setPower(.8);
                }
                else {
                    Robot.swivelmotorRight.setPower(.8);
                    Robot.swivelmotorLeft.setPower(.8);
                }*/
            }
            else if (b_button == true) //med
            {
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorRight.setTargetPosition(MED_POLE);
                Robot.swivelmotorLeft.setTargetPosition(MED_POLE);

                Robot.swivelmotorRight.setPower(.7);
                Robot.swivelmotorLeft.setPower(.7);

               /* if (Robot.swivelmotorRight.getCurrentPosition() > MED_POLE) {
                    Robot.swivelmotorRight.setPower(.25);
                    Robot.swivelmotorLeft.setPower(.25);
                }
                else
                {
                    //power_level = MED_POLE - Robot.swivelmotorRight.getCurrentPosition();
                    Robot.swivelmotorRight.setPower(.8);
                    Robot.swivelmotorLeft.setPower(.8);
                }*/
            }
            else if (a_button == true) //low
            {
                Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.swivelmotorRight.setTargetPosition(LOW_POLE);
                Robot.swivelmotorLeft.setTargetPosition(LOW_POLE);

                Robot.swivelmotorRight.setPower(.7);
                Robot.swivelmotorLeft.setPower(.7);
                /*if (Robot.swivelmotorRight.getCurrentPosition() > LOW_POLE) {
                    Robot.swivelmotorRight.setPower(.25);
                    Robot.swivelmotorLeft.setPower(.25);
                }
                else {
                    Robot.swivelmotorRight.setPower(.7);
                    Robot.swivelmotorLeft.setPower(.7);
                }*/
            }
            else
            {

            }
/*
            if (dpadupSlow) {
                Robot.leftFront.setPower(CREAP_VALUE);
                Robot.leftBack.setPower(CREAP_VALUE);
                Robot.rightFront.setPower(CREAP_VALUE);
                Robot.rightBack.setPower(CREAP_VALUE);

            }
            else if (dpaddownSlow) {
                Robot.leftFront.setPower(-CREAP_VALUE);
                Robot.leftBack.setPower(-CREAP_VALUE);
                Robot.rightFront.setPower(-CREAP_VALUE);
                Robot.rightBack.setPower(-CREAP_VALUE);
            }
            else {
                Robot.leftFront.setPower(0);
                Robot.leftBack.setPower(0);
                Robot.rightFront.setPower(0);
                Robot.rightBack.setPower(0);
            }
            if (dpadleftSlow) {
                Robot.rightFront.setPower(CREAP_VALUE);
                Robot.rightBack.setPower(CREAP_VALUE);
                Robot.leftFront.setPower(-CREAP_VALUE);
                Robot.leftBack.setPower(-CREAP_VALUE);
            }
            else if(dpadrightSlow) {
                Robot.rightFront.setPower(-CREAP_VALUE);
                Robot.rightBack.setPower(-CREAP_VALUE);
                Robot.leftFront.setPower(CREAP_VALUE);
                Robot.leftBack.setPower(CREAP_VALUE);
            }
            else {
                Robot.leftFront.setPower(0);
                Robot.leftBack.setPower(0);
                Robot.rightFront.setPower(0);
                Robot.rightBack.setPower(0);
            }*/


            //Robot.swivelmotorRight.setPower(leftStickY2);

            Robot.liftmotor.setPower(rt_button);
            Robot.liftmotor.setPower(-lt_button);
           /* telemetry.addData("rt_button:", rt_button);
            telemetry.addData("lt_button:", lt_button);
            telemetry.addData("y_button:", y_button);
            telemetry.addData("b_button:", b_button);*/
            telemetry.addData("swivel pos Right:", Robot.swivelmotorRight.getCurrentPosition());
            telemetry.addData("swivel pos Left:", Robot.swivelmotorLeft.getCurrentPosition());
            telemetry.update();

        } //while

    }
}


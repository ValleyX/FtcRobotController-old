package org.firstinspires.ftc.team22076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//This code works with "LEGS" Robot controller

@TeleOp(name="JuniorHighTankDrive")

public class TankDriveControl extends LinearOpMode {
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
        double rightStickY;
        boolean dpadup;
        boolean dpaddown;//swivels the arm up or down
        double lt_button;//lift motor for moving the arm down
        double rt_button;//lift motor control for moving the arm up
        boolean a_button;//grip
        boolean b_button;//realese
        double leftStickY2;

        //Wait for User to push Start on the Controller
        waitForStart();

        //do loop until stop on controller is pressed
        while (opModeIsActive()) {

            leftStickY = -gamepad1.left_stick_y;
            rightStickY = -gamepad1.right_stick_y;
            dpadup = gamepad2.dpad_up;
            dpaddown = gamepad2.dpad_down;
            lt_button = -gamepad2.left_trigger;
            rt_button = gamepad2.right_trigger;
            a_button = gamepad2.a;
            b_button = gamepad2.b;
            //leftStickY2 = -gamepad2.left_stick_y;

          //  System.out.println("ValleyX stickleft " + leftStickY);
            //a button = close
            if (a_button){
                Robot.gripservo.setPosition(0.40); //close
            }
            //b button = open
            if (b_button){
                Robot.gripservo.setPosition(0.05);  //open
            }

            if (dpadup){
                Robot.swivelmotorRight.setPower(1);
                Robot.swivelmotorLeft.setPower(1);
            }
            else if (dpaddown){
                Robot.swivelmotorRight.setPower(-.25);
                Robot.swivelmotorLeft.setPower(-.25);
            }
           else{
              Robot.swivelmotorRight.setPower(0);
              Robot.swivelmotorLeft.setPower(0);

            }
            //Robot.swivelmotor.setPower(leftStickY2);
            Robot.liftmotor.setPower(rt_button);
            Robot.liftmotor.setPower(lt_button);
            Robot.leftFront.setPower(leftStickY);
            Robot.rightFront.setPower(rightStickY);
            Robot.rightBack.setPower(rightStickY);
            Robot.leftBack.setPower(leftStickY);

        }

    }

}

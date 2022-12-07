package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name = "FirstTest2022")
public class FirstTest2022 extends LinearOpMode {

    //declare motors (If YOU FUNKY THIS, DONT!!!)
    DcMotor lFMotor; //port 1
    DcMotor lBMotor; //port 0
    DcMotor rFMotor; //port 3
    DcMotor rBMotor; //port 2


//DO NOT MESS WITH THE ROBOT CONFIGURATION



    @Override
    public void runOpMode() throws InterruptedException {
        //more motor stuff
        lFMotor = hardwareMap.get(DcMotor.class, "lFMotor");
        lBMotor = hardwareMap.get(DcMotor.class, "lBMotor");
        rFMotor = hardwareMap.get(DcMotor.class, "rFMotor");
        rBMotor = hardwareMap.get(DcMotor.class, "rBMotor");

        //inverting motors because drive base is cringe (If YOU FUNKY THIS, DONT!!!)
        lFMotor.setDirection(DcMotor.Direction.REVERSE); //correct
        lBMotor.setDirection(DcMotor.Direction.REVERSE); //correct
        rFMotor.setDirection(DcMotor.Direction.FORWARD); //correct
        rBMotor.setDirection(DcMotor.Direction.FORWARD); //correct


        //stick control variables
        double leftY;
        double rightY;

        //speed control variables
        double leftY2;
        double rightX;

        //strafe control variables
        double leftLever;
        double rightLever;

        //waiting for start (if you are reading this comment, why?)
        waitForStart();


        //contols
        while(opModeIsActive()) {
            //tank drive vars
            //leftY = -gamepad1.left_stick_y;
            //rightY = -gamepad1.right_stick_y;

            leftY2 = -gamepad1.left_stick_y;
            rightX = -gamepad1.right_stick_x;

            leftLever = gamepad1.left_trigger;
            rightLever = gamepad1.right_trigger;

            //left strafe controls
            if(leftLever > 0.5) {
                lFMotor.setPower(-leftLever);
                lBMotor.setPower(leftLever);
                rFMotor.setPower(leftLever);
                rBMotor.setPower(-leftLever);
            }
            //right strafe controls
            else if(rightLever > 0.5) {
                lFMotor.setPower(rightLever);
                lBMotor.setPower(-rightLever);
                rFMotor.setPower(-rightLever);
                rBMotor.setPower(rightLever);
            }

            //basic movements (Tank Drive)
            /*else {
                lFMotor.setPower(leftY);
                lBMotor.setPower(leftY);
                rFMotor.setPower(rightY);
                rBMotor.setPower(rightY);
            }*/

            //turn right???
            else if (rightX >= 0.3 ) {
                lFMotor.setPower(-rightX);
                lBMotor.setPower(-rightX);
                rFMotor.setPower(rightX);
                rBMotor.setPower(rightX);
            }

            //turn left???
            else if (rightX <= -0.3) {
                lFMotor.setPower(-rightX);
                lBMotor.setPower(-rightX);
                rFMotor.setPower(rightX);
                rBMotor.setPower(rightX);
            }

            //forwards
            else if (leftY2 >= 0.3) {
                lFMotor.setPower(leftY2);
                lBMotor.setPower(leftY2);
                rFMotor.setPower(leftY2);
                rBMotor.setPower(leftY2);
            }

            //backwards
            else if (leftY2 <= -0.3) {
                lFMotor.setPower(leftY2);
                lBMotor.setPower(leftY2);
                rFMotor.setPower(leftY2);
                rBMotor.setPower(leftY2);
            }

            else {
                lFMotor.setPower(0);
                lBMotor.setPower(0);
                rFMotor.setPower(0);
                rBMotor.setPower(0);


            }

            telemetry.addData("leftY2", leftY2);
            telemetry.addData("rightX", rightX);
            telemetry.update();



        }

    }
}

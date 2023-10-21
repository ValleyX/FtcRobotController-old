package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestCodeArcade")
public class TestCodeArcade extends LinearOpMode {

    //declares the motors
    public DcMotor lFMotor;
    public DcMotor lBMotor;
    public DcMotor rFMotor;
    public DcMotor rBMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        lFMotor = hardwareMap.get(DcMotor.class, "lFMotor");
        lBMotor = hardwareMap.get(DcMotor.class, "lBMotor");
        rFMotor = hardwareMap.get(DcMotor.class, "rFMotor");
        rBMotor = hardwareMap.get(DcMotor.class, "rBMotor");

        // define stick x/y
        double lefty;
        double rightx;

        //wait...............go
        waitForStart();

        //controls
        while (opModeIsActive()) {
            //makes the variables for the sticks
            lefty = -gamepad1.left_stick_y;
            rightx = -gamepad1.right_stick_x;

            //set the power to the motors

            lFMotor.setPower(lefty);
            lBMotor.setPower(lefty);
            rFMotor.setPower(-lefty);
            rBMotor.setPower(-lefty);


            lFMotor.setPower(rightx);
            lBMotor.setPower(rightx);
            rFMotor.setPower(rightx);
            rBMotor.setPower(rightx);

        }
    }
}
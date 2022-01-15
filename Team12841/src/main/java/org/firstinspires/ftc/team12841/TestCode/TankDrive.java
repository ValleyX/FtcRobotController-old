package org.firstinspires.ftc.team12841.TestCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team12841.Drivers.LiftDrive;

@TeleOp (name = "tank")
public class TankDrive extends LinearOpMode {

    private final double LiftCOUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    private final double LiftDRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    private final double LiftONE_MOTOR_COUNT_IN_ONE_REV = LiftCOUNTS_PER_MOTOR_REV * LiftDRIVE_GEAR_REDUCTION;
    private final double LiftWHEEL_CIRCUMFRANSE = 1.5;
    private final double LiftINCHES_IN_ONE_REV = LiftWHEEL_CIRCUMFRANSE * Math.PI;
    final double LiftCOUNTS_PER_INCH = LiftONE_MOTOR_COUNT_IN_ONE_REV / LiftINCHES_IN_ONE_REV; //TODO determine in class

    @Override
    public void runOpMode() throws InterruptedException {

        //declare motors
        //DT Motors
        DcMotor LfMotor;
        DcMotor LbMotor;
        DcMotor RfMotor;
        DcMotor RbMotor;
        //ducky wheel motors
        DcMotor SpinnerMotor;
        //Lift Motors
        DcMotor LiftMotor;
        DcMotor InMotor;
        Servo InServo;


        //more DT motor stuff
        LfMotor = hardwareMap.get(DcMotor.class, "LfMotor");
        RfMotor = hardwareMap.get(DcMotor.class, "RfMotor");
        LbMotor = hardwareMap.get(DcMotor.class, "LbMotor");
        RbMotor = hardwareMap.get(DcMotor.class, "RbMotor");

        LfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LbMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //more lift motor stuff
       LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
       LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       InMotor = hardwareMap.get(DcMotor.class, "InMotor");
       InServo = hardwareMap.get(Servo.class, "InServo");

        //spinner motor stuff
        SpinnerMotor = hardwareMap.get(DcMotor.class, "SpinnerMotor");
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //movement

        while (opModeIsActive()) {
            double lyOne = gamepad1.left_stick_y;
            double ryOne = gamepad1.right_stick_y;
            boolean rbOne = gamepad1.right_bumper;
            boolean lbOne = gamepad1.left_bumper;
            double lyTwo = gamepad2.left_stick_y;
            boolean aButtonTwo = gamepad2.a;
            boolean lbTwo = gamepad2.left_bumper;
            boolean rbTwo = gamepad2.right_bumper;


            LfMotor.setPower(lyOne);
            LbMotor.setPower(lyOne);
            RfMotor.setPower(ryOne);
            RbMotor.setPower(ryOne);
           //  LiftMotor.setPower(lyTwo);


            if (aButtonTwo){
                InServo.setPosition(0.65);
            }else {
                InServo.setPosition(0.8);
            }

            if ((lyTwo > .5) && (LiftMotor.getCurrentPosition() > (0 * LiftCOUNTS_PER_INCH)))
            {
                LiftMotor.setPower(-.5);
            } else if ((lyTwo < -.5) && (LiftMotor.getCurrentPosition() < (13 * LiftCOUNTS_PER_INCH))){
                LiftMotor.setPower(.5);
            } else if ((lyTwo < 0) && (LiftMotor.getCurrentPosition() > (0 * LiftCOUNTS_PER_INCH))){
                LiftMotor.setPower(lyTwo);
            } else if ((lyTwo > 0) && (LiftMotor.getCurrentPosition() < (13 * LiftCOUNTS_PER_INCH))){
                LiftMotor.setPower(lyTwo);
            } else {
                LiftMotor.setPower(0);
            }

            if (lbTwo) {
                InMotor.setPower(-.75);
            } else if (rbTwo){
                InMotor.setPower(.75);
            } else {
                InMotor.setPower(0);
            }


            if (rbOne) {
                SpinnerMotor.setPower(-1);
            } else if (lbOne){
                SpinnerMotor.setPower(1);
            } else {
                SpinnerMotor.setPower(0);
            }

            telemetry.addData("Path0", "lift position at %7d ",
                    LiftMotor.getCurrentPosition());
           // robot_.OpMode_.telemetry.update();
                telemetry.addData("LeftStickYDrive = ", lyOne);
                telemetry.addData("RightStickYDrive = ", ryOne);
                telemetry.addData("RightBumperDuck = ", rbOne);
                telemetry.addData("LeftBumperDuck = ", lbOne);
                telemetry.addData("LeftStickYLift = ", lyTwo);
                telemetry.addData("lift encoder ", LiftMotor.getCurrentPosition());
                telemetry.addData( "AButton = ", aButtonTwo);
                telemetry.addData("RightBumperIn = ", rbTwo);
                telemetry.addData("LeftBumperIn = ", lbTwo);

                telemetry.update();

            }
    }
}
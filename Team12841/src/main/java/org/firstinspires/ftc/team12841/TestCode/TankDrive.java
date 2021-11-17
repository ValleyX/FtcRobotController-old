package org.firstinspires.ftc.team12841.TestCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "tank")
public class TankDrive extends LinearOpMode {
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
        //DcMotor LiftMotor;
        DcMotor InMotor;//


        //more DT motor stuff
        LfMotor = hardwareMap.get(DcMotor.class, "LfMotor");
        RfMotor = hardwareMap.get(DcMotor.class, "RfMotor");
        LbMotor = hardwareMap.get(DcMotor.class, "LbMotor");
        RbMotor = hardwareMap.get(DcMotor.class, "RbMotor");

        LfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LbMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //more lift motor stuff
       // LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
       // InMotor = hardwareMap.get(DcMotor.class, "InMotor");

        //spinner motor stuff
        SpinnerMotor = hardwareMap.get(DcMotor.class, "SpinnerMotor");

        waitForStart();

        //movement

        while (opModeIsActive()) {
            double lyOne = gamepad1.left_stick_y;
            double ryOne = gamepad1.right_stick_y;
            boolean rbOne = gamepad1.right_bumper;

            LfMotor.setPower(lyOne);
            LbMotor.setPower(lyOne);
            RfMotor.setPower(ryOne);
            RbMotor.setPower(ryOne);

            if (rbOne) {
                SpinnerMotor.setPower(.75);
            } else {
                SpinnerMotor.setPower(0);
            }

            telemetry.addData("LeftStickY = ", lyOne);
            telemetry.addData("RightStickY = ", ryOne);
            telemetry.addData("RightBumper = ", rbOne);
            telemetry.update();

        }
    }
}
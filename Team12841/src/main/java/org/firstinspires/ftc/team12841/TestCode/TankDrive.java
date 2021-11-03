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
        //Lift Motors
        DcMotor LiftMotor;
        DcMotor InMotor;


        //more DT motor stuff
        LfMotor = hardwareMap.get(DcMotor.class, "LfMotor");
        RfMotor = hardwareMap.get(DcMotor.class, "RfMotor");
        LbMotor = hardwareMap.get(DcMotor.class, "LbMotor");
        RbMotor = hardwareMap.get(DcMotor.class, "RbMotor");

        LfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LbMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //more lift motor stuff
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        InMotor = hardwareMap.get(DcMotor.class, "InMotor");


        waitForStart();

        //movement

        while (opModeIsActive()) {
            double ly = gamepad1.left_stick_y;
            double ry = gamepad1.right_stick_y;

            LfMotor.setPower(ly);
            LbMotor.setPower(ly);
            RfMotor.setPower(ry);
            RbMotor.setPower(ry);

            //LiftMotor.setPower()

            telemetry.addData("LeftStickY = ",ly);
            telemetry.addData("RightStickY = ",ry);
            telemetry.update();

        }
    }
}
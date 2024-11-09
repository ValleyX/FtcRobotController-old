package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;

@TeleOp(name = "lift reset")

public class liftResetManual extends LinearOpMode {
    RobotHardware robot_;
    @Override
    public void runOpMode() throws InterruptedException {
        robot_ = new RobotHardware(this);


        waitForStart();




        // run the scheduler
        //TODO; add telemetry into here if not already in commands or subsystems
        while (!isStopRequested() && opModeIsActive()) {



            //temporary
            if((gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0) /*&& robot_.liftMotor.getCurrentPosition() >= 0*/){
                robot_.liftMotor.setPower(-gamepad2.left_stick_y);
            } else {
                robot_.liftMotor.setPower(0);
            }
            telemetry.addData("current pos", robot_.liftMotor.getCurrentPosition());
            telemetry.update();

        }

    }
}

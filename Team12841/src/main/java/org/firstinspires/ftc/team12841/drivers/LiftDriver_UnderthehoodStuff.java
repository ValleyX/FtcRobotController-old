package org.firstinspires.ftc.team12841.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftDriver_UnderthehoodStuff {

    RobotHardware robotHardware_;
    ElapsedTime timeRunning;


    public LiftDriver_UnderthehoodStuff(RobotHardware robotHardware) {
        robotHardware_ = robotHardware;
        timeRunning = new ElapsedTime();
        robotHardware_.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.liftMotor.setTargetPosition(0);
        robotHardware_.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveInches(double height, double speed)
    {
        robotHardware_.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Making the motors read the clicks
      //  robotHardware_.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // robotHardware_.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotHardware_.liftMotor.setTargetPosition((int) (height * robotHardware_.liftDistancePerRev));

       // robotHardware_.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.liftMotor.setPower(speed);
    }
}

package org.firstinspires.ftc.team12841.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderDrive_UnderthehoodStuff {

    RobotHardware robotHardware_;
    ElapsedTime timeRunning;


    public EncoderDrive_UnderthehoodStuff (RobotHardware robotHardware) {
        robotHardware_ = robotHardware;
        timeRunning = new ElapsedTime();
    }

    public void moveInches(double lInches, double rInches, double speed, double timeOut)
    {
        //Setting the Variables
        int currentPositionlBMotor;
        int currentPositionlFMotor;
        int currentPositionrBMotor;
        int currentPositionrFMotor;

        //Making the motors read the clicks (i think)
        robotHardware_.lBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.lFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.rBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.rFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.lBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.lFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.rBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.rFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPositionlBMotor = robotHardware_.lBMotor.getCurrentPosition();
        currentPositionlFMotor = robotHardware_.lFMotor.getCurrentPosition();
        currentPositionrBMotor = robotHardware_.rBMotor.getCurrentPosition();
        currentPositionrFMotor = robotHardware_.rFMotor.getCurrentPosition();
        double clicksToMoveL = lInches * robotHardware_.distancePerRev;
        System.out.println("ValleyX clicksToMoveL " + clicksToMoveL);
        double clicksToMoveR = rInches * robotHardware_.distancePerRev;
        System.out.println("ValleyX clicksToMoveL " + clicksToMoveL);
        robotHardware_.lBMotor.setTargetPosition((int) (currentPositionlBMotor + clicksToMoveL));
        robotHardware_.lFMotor.setTargetPosition((int) (currentPositionlFMotor + clicksToMoveL));
        robotHardware_.rBMotor.setTargetPosition((int) (currentPositionrBMotor + clicksToMoveR));
        robotHardware_.rFMotor.setTargetPosition((int) (currentPositionrFMotor + clicksToMoveR));

        robotHardware_.lBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.lFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.rBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.rFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.lBMotor.setPower(speed);
        robotHardware_.lFMotor.setPower(speed);
        robotHardware_.rBMotor.setPower(speed);
        robotHardware_.rFMotor.setPower(speed);

        timeRunning.reset();

        System.out.println("ValleyX  entering while loop");
        System.out.println("ValleyX lBMotor is busy " + robotHardware_.lBMotor.isBusy());
        System.out.println("ValleyX lFMotor is busy " + robotHardware_.lFMotor.isBusy());
        System.out.println("ValleyX rBMotor is busy " + robotHardware_.rBMotor.isBusy());
        System.out.println("ValleyX rFMotor is busy " + robotHardware_.rFMotor.isBusy());
        while((robotHardware_.lBMotor.isBusy() && robotHardware_.lFMotor.isBusy() && robotHardware_.rBMotor.isBusy() && robotHardware_.rFMotor.isBusy())
         && timeRunning.seconds() < timeOut && robotHardware_.opMode_.opModeIsActive())
        {
            robotHardware_.opMode_.idle();
        }
        System.out.println("ValleyX  exiting while loop");
    }
}
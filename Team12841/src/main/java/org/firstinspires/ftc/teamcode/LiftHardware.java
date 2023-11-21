package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftHardware {
    RobotHardware robotHardware_;
    ElapsedTime timeRunning;

    public boolean intake = false;
    public boolean outtake = false;
    public final double countsPerMotorRev = 28.0;
    public final double elbowDriveGearReduction = 10.0;
    public final double elevatorDriveGearReduction = 10.0;
    public final double elbowMotorEncoderCountsPerRev = countsPerMotorRev * elbowDriveGearReduction;
    public final double elevatorMotorEncoderCountsPerRev = countsPerMotorRev * elevatorDriveGearReduction;
    public final double liftDiameterMotor = 1.504;
    public final double degreesPerClick = 360 / elbowMotorEncoderCountsPerRev;
    public final double elevatorInches = liftDiameterMotor * Math.PI;

    public final double elevatorDistancePerRev = elevatorMotorEncoderCountsPerRev / elevatorInches;
    public LiftHardware(RobotHardware robotHardware) {
        robotHardware_ = robotHardware;
        timeRunning = new ElapsedTime();
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.elbowMotor.setTargetPosition(0);
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.elevatorMotor.setTargetPosition(0);
        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }

    public void moveElbow(double degree, double speed)
    {
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Making the motors read the clicks

        robotHardware_.elbowMotor.setTargetPosition((int) (degree * degreesPerClick));

        robotHardware_.elbowMotor.setPower(speed);
    }

    public void moveElevator( double inch, double speed)
    {
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Making the motors read the clicks

        robotHardware_.elbowMotor.setTargetPosition((int) (inch * elevatorDistancePerRev));

        robotHardware_.elbowMotor.setPower(speed);
    }

    //method that turns to intake on
    public void startIntake()
    {
        intake = true;
        outtake = false;
    }
    //method that turns the intake off yeet
    public void stopIntake()
    {
        intake = false;
        outtake = false;
    }

    public void reverseIntake()
    {
        intake = false;
        outtake = true;
    }

}

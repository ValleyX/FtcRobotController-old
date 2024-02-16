package RobotHardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import RobotHardwares.RobotHardware;

public class LiftHardware {
    RobotHardware robotHardware_;
    ElapsedTime timeRunning;

    LinearOpMode opMode_;

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
    public LiftHardware(RobotHardware robotHardware, LinearOpMode opMode) {
        robotHardware_ = robotHardware;
        opMode_ = opMode;
        timeRunning = new ElapsedTime();
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.elbowMotor.setTargetPosition(0);
        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.elevatorMotor.setTargetPosition(0);
        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveElbow(int pos, double speed)
    {
        robotHardware_.elbowMotor.setTargetPosition(pos);

        robotHardware_.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotHardware_.elbowMotor.setPower(speed);

        //robotHardware_.elbowMotor.is

        //Making the motors read the clicks
    }

    public void moveElevator(int pos, double speed)
    {
        robotHardware_.elevatorMotor.setTargetPosition(-pos);

        robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotHardware_.elevatorMotor.setPower(speed);

        //robotHardware_.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Making the motors read the clicks
    }

    public void closeBucket()
    {
        robotHardware_.bucket.setPosition(0.6);
    } //close

    public void openBucket()
    {
        robotHardware_.bucket.setPosition(0.7);
    } //open


}

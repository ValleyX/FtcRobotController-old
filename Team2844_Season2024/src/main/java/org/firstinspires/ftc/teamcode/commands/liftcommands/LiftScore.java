package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This hooks specimins on chamber bar */
public class LiftScore extends CommandBase {
    LiftSubsystem m_liftSub;
    DriveSubsystem m_driveSub;
    LinearOpMode m_opMode;
    double currentPosInches;
    public LiftScore(LiftSubsystem liftSubsystem, DriveSubsystem driveSub, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_driveSub = driveSub;
        m_opMode = opMode;

        currentPosInches = m_liftSub.m_liftMotor.getCurrentPosition() / RobotHardware.LIFT_COUNTS_PER_INCH;

    }


    @Override
    public void initialize(){
        currentPosInches = m_liftSub.m_liftMotor.getCurrentPosition() / RobotHardware.LIFT_COUNTS_PER_INCH;
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_ENGAGED);
        m_opMode.sleep(500);
        m_liftSub.liftToPosition(currentPosInches-3,1);
        m_opMode.sleep(700);
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_DISENGAGED);
        m_opMode.sleep(100);
        m_driveSub.allPower(-.5);
        m_opMode.sleep(200);
        m_driveSub.allPower(0);
        m_liftSub.liftToPosition(0,1);


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

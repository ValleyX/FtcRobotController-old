package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/** Resets lift to original position*/
public class LiftReset extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftReset(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.liftToPosition(0,1);
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_DISENGAGED);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
         return true;
     }
}


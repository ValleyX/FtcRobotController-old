package org.firstinspires.ftc.teamcode.commands.hangcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/** sets hang to original  position*/
public class HangReset extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public HangReset(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.hangToPosition(0,1);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

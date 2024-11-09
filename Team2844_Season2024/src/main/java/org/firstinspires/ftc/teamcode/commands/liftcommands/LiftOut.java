package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This puts the lift out at the start of the hang sequence*/
public class LiftOut extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftOut(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.liftToPosition(6.5,1);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This puts lift to high chamber scoring position*/
public class LiftHighChamber extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftHighChamber(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //TODO; put in actual value for the position
        m_liftSub.liftToPosition(17,1);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

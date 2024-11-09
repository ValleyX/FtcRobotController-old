package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This pulls the lift in at the second stage of the hang sequence*/
public class LiftIn extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftIn(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.liftToPosition(2.5,1);


    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        if(m_liftSub.m_liftMotor.isBusy()){
            return false;
        } else {
            return true;
        }
    }
}

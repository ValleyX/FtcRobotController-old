package org.firstinspires.ftc.teamcode.commands.hangcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This puts the hang out at the start of the hang sequence*/
public class HangStart extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public HangStart(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_liftSub.hangToPosition(15,1);
        //m_opMode.sleep(3000);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        if(m_liftSub.m_hangMotor.isBusy()){
            return false;
        } else {
            return true;
        }
    }
}

package org.firstinspires.ftc.teamcode.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**Turns on intake*/
public class SubExtendIntake extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendIntake(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_intakeSub.intakeOn();
    }

    @Override
    public void end(boolean interrupted){
        m_intakeSub.intakeOff();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
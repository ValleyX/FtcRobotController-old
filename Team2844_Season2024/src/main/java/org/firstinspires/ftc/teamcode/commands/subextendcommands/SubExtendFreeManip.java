package org.firstinspires.ftc.teamcode.commands.subextendcommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**free movement tof subextend to a joystick*/
public class SubExtendFreeManip extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendFreeManip(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double rt = m_opMode.gamepad1.right_trigger;
        double lt = m_opMode.gamepad1.right_trigger;
        if(rt > 0){
            m_intakeSub.m_SubExtend.setPower(rt);
        } else if (lt > 0) {
            m_intakeSub.m_SubExtend.setPower(-lt);
        }
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

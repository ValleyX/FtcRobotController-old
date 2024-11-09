package org.firstinspires.ftc.teamcode.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**This sends the subextend out including putting the bucket out*/
public class SubExtendOut extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendOut(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_intakeSub.setIntakeDropServo(RobotHardware.DROP_SERVO_DOWN);
        //TODO: put in real value
        m_intakeSub.subExtendToPosition(2.5,1);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

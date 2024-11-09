package org.firstinspires.ftc.teamcode.commands.subextendcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**sequence to drop specimin off at  human player station*/
public class SubExtendHumanPlayer extends CommandBase {
    IntakeSubsystem m_intakeSub;
    LinearOpMode m_opMode;
    public SubExtendHumanPlayer(IntakeSubsystem intakeSubsystem, LinearOpMode opMode){
        m_intakeSub = intakeSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        m_intakeSub.setIntakeDropServo(RobotHardware.DROP_SERVO_DOWN);

        m_intakeSub.intakeExtake();
        m_opMode.sleep(5000);

        m_intakeSub.intakeOff();

        m_intakeSub.setIntakeDropServo(RobotHardware.DROP_SERVO_UP);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

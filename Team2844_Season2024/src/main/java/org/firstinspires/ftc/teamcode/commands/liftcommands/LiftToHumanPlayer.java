package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**This lets the lift collect specimins from the human player*/
public class LiftToHumanPlayer extends CommandBase {
    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;
    public LiftToHumanPlayer(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        m_liftSub = liftSubsystem;
        m_opMode = opMode;

    }


    @Override
    public void initialize(){
        //TODO; put in actual value for the position
        m_liftSub.liftToPosition(2.5,1);
        m_opMode.sleep(500);
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_ENGAGED);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
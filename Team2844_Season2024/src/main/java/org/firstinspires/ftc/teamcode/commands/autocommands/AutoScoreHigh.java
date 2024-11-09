package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class AutoScoreHigh extends CommandBase {

    LiftSubsystem m_liftSub;
    LinearOpMode m_opMode;

    public AutoScoreHigh(LiftSubsystem liftSub, LinearOpMode opMode) {

        m_liftSub = liftSub;
        m_opMode = opMode;

    }

    @Override
    public void initialize(){
        m_liftSub.liftToPosition(17,1);
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_ENGAGED);
        m_opMode.sleep(700);
        m_liftSub.liftToPosition(13,1);
        m_opMode.sleep(700);
        m_liftSub.bucketToPosition(RobotHardware.BUCKET_SERVO_DISENGAGED);
        m_opMode.sleep(100);
    }

}

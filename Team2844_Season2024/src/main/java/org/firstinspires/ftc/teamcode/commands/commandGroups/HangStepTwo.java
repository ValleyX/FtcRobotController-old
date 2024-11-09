package org.firstinspires.ftc.teamcode.commands.commandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.hangcommands.HangExtend;
import org.firstinspires.ftc.teamcode.commands.hangcommands.HangOut;
import org.firstinspires.ftc.teamcode.commands.hangcommands.HangReset;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftIn;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftOut;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/** second step of hang sequence*/
public class HangStepTwo extends SequentialCommandGroup {
    public HangStepTwo(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        addCommands(
                new HangExtend(liftSubsystem,opMode),
                new LiftIn(liftSubsystem,opMode),
                new ParallelCommandGroup(
                        new LiftReset(liftSubsystem,opMode),
                        new HangReset(liftSubsystem, opMode)
                )
        );

        addRequirements(liftSubsystem);
    }
}

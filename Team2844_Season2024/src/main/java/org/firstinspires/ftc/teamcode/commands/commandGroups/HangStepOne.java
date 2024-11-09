package org.firstinspires.ftc.teamcode.commands.commandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.hangcommands.HangOut;
import org.firstinspires.ftc.teamcode.commands.hangcommands.HangStart;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftOut;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**first step of the hang sequence*/
public class HangStepOne extends SequentialCommandGroup {
    public HangStepOne(LiftSubsystem liftSubsystem, LinearOpMode opMode){
        addCommands(
               new HangStart(liftSubsystem,opMode),
                new HangOut(liftSubsystem,opMode),
                new LiftOut(liftSubsystem,opMode)


        );

        addRequirements(liftSubsystem);
    }
}

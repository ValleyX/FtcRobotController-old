package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class DropIntake extends CommandBase {
    IntakeSubsystem m_intake;
    LinearOpMode m_opmode;

    public DropIntake(IntakeSubsystem intake, LinearOpMode opmode) {
        m_intake = intake;
        m_opmode = opmode;

    }


    @Override
    public void initialize() {
        m_intake.setIntakeDropServo(RobotHardware.INTAKE_SERVO_DOWN);
        m_intake.intakeOn();
    }
}

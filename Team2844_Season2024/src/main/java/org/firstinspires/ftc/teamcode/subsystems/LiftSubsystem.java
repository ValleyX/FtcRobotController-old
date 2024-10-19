package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftSubsystem extends SubsystemBase {
    public DcMotor m_liftMotor;
    public DcMotor m_hangMotor;
    public Servo m_bucket;
    public LiftSubsystem(DcMotor liftMotor, DcMotor hangMotor, Servo bucket){
        m_liftMotor = liftMotor;
        m_hangMotor = hangMotor;
        m_bucket = bucket;
    }

    @Override
    public void periodic(){

    }
}
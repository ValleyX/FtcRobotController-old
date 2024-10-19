package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

//Drive Subsystem
public class DriveSubsystem extends SubsystemBase {
    public DcMotor m_frontLeft;
    public DcMotor m_frontRight;
    public DcMotor m_backLeft;
    public DcMotor m_backRight;
    public IMU m_imu;
    public DriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu){
        m_frontLeft = frontLeft;
        m_backLeft = backLeft;
        m_backRight = backRight;
        m_frontRight = frontRight;
        m_imu = imu;
    }

    @Override
    public void periodic(){

    }
}
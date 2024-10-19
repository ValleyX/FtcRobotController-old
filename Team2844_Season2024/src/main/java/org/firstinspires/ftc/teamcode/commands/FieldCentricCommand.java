package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


public class FieldCentricCommand extends CommandBase {

    private final DriveSubsystem m_driveSubsystem;
    public LinearOpMode m_opMode;
    public FieldCentricCommand(DriveSubsystem driveSubsystem, LinearOpMode opMode){
        m_driveSubsystem = driveSubsystem;
        m_opMode = opMode;
        addRequirements(m_driveSubsystem); //prevents other command from using this subsysten while active
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double y = -m_opMode.gamepad1.left_stick_y; // Remember, this is reversed!
        double x = m_opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = m_opMode.gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        //double botHeading = -m_driveSubsytem.m_imu.getAngularOrientation().firstAngle;
        double botHeading = -m_driveSubsystem.m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        m_driveSubsystem.m_frontLeft.setPower(frontLeftPower);
        m_driveSubsystem.m_backLeft.setPower(backLeftPower);
        m_driveSubsystem.m_frontRight.setPower(frontRightPower);
        m_driveSubsystem.m_backRight.setPower(backRightPower);

        m_opMode.telemetry.addData("rotX", rotX);
        m_opMode.telemetry.addData("rotY", rotY);
        m_opMode.telemetry.addData("front left power", frontLeftPower);
        m_opMode.telemetry.addData("front right power", frontRightPower);
        m_opMode.telemetry.addData("back left power", backLeftPower);
        m_opMode.telemetry.addData("back right power", backRightPower);
        m_opMode.telemetry.addData("denominator", denominator);
        m_opMode.telemetry.update();
    }

    // @Override
    //public boolean isFinished(){
    //     return true;
    // }
}
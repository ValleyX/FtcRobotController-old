package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {
    LinearOpMode opMode_;
    public DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    public DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    public DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    public DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    public Camera camera = null;
    public AprilTagCamera aprilTagCamera = null;
    RobotHardware(LinearOpMode opMode)
    {
        opMode_ = opMode;
        leftFrontDrive  = opMode_.hardwareMap.get(DcMotor.class, "lFMotor");
        rightFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "rFMotor");
        leftBackDrive  = opMode_.hardwareMap.get(DcMotor.class, "lBMotor");
        rightBackDrive = opMode_.hardwareMap.get(DcMotor.class, "rBMotor");
        camera = new Camera(opMode_);
        aprilTagCamera = new AprilTagCamera(opMode_,this);

    }

}
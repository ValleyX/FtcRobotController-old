package org.firstinspires.ftc.teamcode.Drivers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    public LinearOpMode OpMode_; // pointer to the run time operation mode
    public final double deadband = 0.2; //later move to RobotHardware or place with all the constants


    // Adjust these numbers to suit your robot.
    public final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public final double SPEED_GAIN  =  0.25  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN =  0.1 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public final double MAX_AUTO_SPEED = 1;   // 0.5 Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public final double OD_COUNTS_PER_MOTOR_REV = 8192;    //  AndyMark Motor Encoder
    public final double OD_DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public final double OD_ONE_MOTOR_COUNT = OD_COUNTS_PER_MOTOR_REV * OD_DRIVE_GEAR_REDUCTION;
    public final double OD_Distance_in_one_rev = 2.0 * Math.PI; //in
    public final double OD_COUNTS_PER_INCH = OD_ONE_MOTOR_COUNT / OD_Distance_in_one_rev;


    public static boolean findTag = false; //if this finds the tag, then we use it to turn on/off driving with sticks

    public static double delayTimer = 2000; //delay timer for detection



    //make motors
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;

    //odometry encoders
    public DcMotor verticalLeft;
    public DcMotor verticalRight;
    public DcMotor horizontal;

    public BNO055IMU imu;


    public RobotHardware(LinearOpMode opMode) {
        OpMode_ = opMode;

        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = OpMode_.hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        motorBackRight = OpMode_.hardwareMap.dcMotor.get("rightBack");

        //Declare Odometry encoders
        //make sure they match the names of the motors they are linked to
        verticalLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        verticalRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        horizontal = OpMode_.hardwareMap.dcMotor.get("leftBack");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
       // motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


    }

    //set power to the entire robot
    public void allpower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void leftPower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void rightPower(double power) {
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }
}

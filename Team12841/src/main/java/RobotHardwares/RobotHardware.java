package RobotHardwares;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DoubleTeleOpDrive;

public class RobotHardware {
    //This does all the math for encoder drive
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final ElapsedTime runtime = new ElapsedTime();

    double strafeCorrectionFront = 0.33;
    final double STRAFE_CORRECTION_BACK = 0.9;

    LinearOpMode opMode_;

    //all of the hardware declarations
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor elbowMotor = null; //Motor used to move the lift
    public DcMotor elevatorMotor = null;
    public DcMotor intakeMotor = null;
    public Servo bucket = null;
    public Camera camera = null;
    public Servo airplane_launcher = null;
    public AprilTagCamera aprilTagCamera = null;
    NavxMicroNavigationSensor navxMicro;
    IntegratingGyroscope gyro;
    public RevBlinkinLedDriver lights;


    //all of the drive-number declaration
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTargetF = 0;
    private int leftTargetB = 0;
    private int rightTargetF = 0;
    private int rightTargetB = 0;
    double yawOffSet = 0;

    // How close must the heading get to the target before moving to next step when turning
    static final double HEADING_THRESHOLD = 1; //Larger is less accurate, but smaller takes to much time

    //the gains for the auto turn and drive commands
    static final double P_TURN_GAIN = 0.035; // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.001; // Larger is makes it snake, but smaller makes it drift
    Camera.SkystoneDeterminationPipeline.RobotPos side_;

    public RobotHardware(LinearOpMode opMode, Camera.SkystoneDeterminationPipeline.RobotPos side) {
        side_ = side;
        opMode_ = opMode;

        //mapping motors
        leftFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "lFMotor");//2
        rightFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "rFMotor");//1
        leftBackDrive = opMode_.hardwareMap.get(DcMotor.class, "lBMotor");//3
        rightBackDrive = opMode_.hardwareMap.get(DcMotor.class, "rBMotor");//0
        elbowMotor = opMode_.hardwareMap.get(DcMotor.class, "elbowMotor");
        elevatorMotor = opMode_.hardwareMap.get(DcMotor.class, "elevatorMotor");
        intakeMotor = opMode_.hardwareMap.get(DcMotor.class, "intakeMotor");
        bucket = opMode_.hardwareMap.get(Servo.class, "bucket");
        airplane_launcher = opMode_.hardwareMap.get(Servo.class,"airplane_launcher");
        lights = opMode_.hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        //reverses the right motors to go straight
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = new Camera(opMode_, side_);
        aprilTagCamera = new AprilTagCamera(opMode_, this);

        //setting drive activities
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //make sure you actually use the encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //same thing for the elbow motor
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //same thing for the elevator
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initializes the imu
        navxMicro = opMode_.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode_.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTargetF = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftTargetB = leftBackDrive.getCurrentPosition() + moveCounts;
            rightTargetF = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightTargetB = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftTargetF);
            leftBackDrive.setTargetPosition(leftTargetB);
            rightFrontDrive.setTargetPosition(rightTargetF);
            rightBackDrive.setTargetPosition(rightTargetB);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode_.opModeIsActive() &&
                    (leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode_.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);

            System.out.println("Valleyx: Heading error " + headingError);
            opMode_.sleep(2);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opMode_.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        System.out.println("getSteeringCorrection " + headingError);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        //leftSpeed  = drive - turn;
        //rightSpeed = drive + turn;
        leftSpeed = drive + turn;
        rightSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightBackDrive.setPower(rightSpeed);

    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            opMode_.telemetry.addData("Motion", "Drive Straight");
            opMode_.telemetry.addData("Target Pos LF:LB:RF:RB", "%7d:%7d:%7d:%7d", leftTargetF, leftTargetB, rightTargetF, rightTargetB);
            opMode_.telemetry.addData("Actual Pos LF:LB:RF:RB", "%7d:%7d:%7d:%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        } else {
            opMode_.telemetry.addData("Motion", "Turning");
        }

        opMode_.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        opMode_.telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        opMode_.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        opMode_.telemetry.update();
    }

    public double getHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }

    public double getHeadingRadians() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle;
    }

    public void strafe(double speed)
    {
        if(Math.abs(speed) < 0.2)
        {
            leftBackDrive.setPower(speed);
            leftFrontDrive.setPower(-speed);//***
            rightFrontDrive.setPower(speed); //***
            rightBackDrive.setPower(-speed); //***
        }
        else {
            if(speed < 0)
            {
                strafeCorrectionFront = 0.3;
            }
            else {
                strafeCorrectionFront = 0.33;
            }
            leftBackDrive.setPower(speed);
            opMode_.sleep(100);
            leftFrontDrive.setPower(-speed*strafeCorrectionFront);//***
            rightFrontDrive.setPower(speed*strafeCorrectionFront); //***
            rightBackDrive.setPower(-speed*STRAFE_CORRECTION_BACK); //*** Zach!! what does this mean(the stars)
        }
    }

    public void allPower(double speed)
    {
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
    }

    public void turnBot(double speed)
    {
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        rightBackDrive.setPower(-speed);
    }
}
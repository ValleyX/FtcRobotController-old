package RobotHardwares;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class RobotHardware {
    LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    //TURN MOTOR ENCODER COUNTS TO INCHES
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    //FRONT IS THE U SHAPE AND WHERE THE SLIDES COME OUT
    public DcMotor rightBackDrive = null;//0
    public DcMotor rightFrontDrive = null;//1
    public DcMotor leftFrontDrive = null;//2
    public DcMotor leftBackDrive = null;//3

    public DcMotor pivotMotor = null;
    public DcMotor ySlidesMotorLeft = null;
    public DcMotor ySlidesMotorRight = null;
    public DcMotor extendMotor = null;

    public Servo claw = null;
    public Servo clawWrist = null;

    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double leftFSpeed = 0;
    private double leftBSpeed = 0;
    private double rightFSpeed = 0;
    private double rightBSpeed = 0;
    private int leftTargetF = 0;
    private int leftTargetB = 0;
    private int rightTargetF = 0;
    private int rightTargetB = 0;


    static final double HEADING_THRESHOLD = 1.75; //Larger is less accurate, but smaller takes to much time

    //the gains for the auto turn and drive commands
    static final double P_TURN_GAIN = 0.015; // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.001; // Larger is makes it snake, but smaller makes it drift

    double strafeCorrectionFront = 0.33;
    final double STRAFE_CORRECTION_BACK = 0.9;

//    NavxMicroNavigationSensor navxMicro;
//    IntegratingGyroscope gyro;
    public RevBlinkinLedDriver lights;
//    NormalizedColorSensor colorSensor;
//    NormalizedRGBA colors;
//    final float[] hsvValues = new float[3];
//    float gain = 3.5F;
    public IMU imu;





    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;
        //mapping motors
        rightBackDrive = opMode_.hardwareMap.get(DcMotor.class, "rBMotor");//0
        rightFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "rFMotor");//1
        leftFrontDrive = opMode_.hardwareMap.get(DcMotor.class, "lFMotor");//2
        leftBackDrive = opMode_.hardwareMap.get(DcMotor.class, "lBMotor");//3
        ySlidesMotorLeft = opMode_.hardwareMap.get(DcMotor.class, "ySlidesLeft");
        ySlidesMotorRight = opMode_.hardwareMap.get(DcMotor.class, "ySlidesRight");
        pivotMotor = opMode_.hardwareMap.get(DcMotor.class, "pivotMotor");
        extendMotor = opMode_.hardwareMap.get(DcMotor.class, "extendMotor");

        claw = opMode_.hardwareMap.get(Servo.class, "claw");
        clawWrist = opMode_.hardwareMap.get(Servo.class, "clawWrist");
        clawWrist.getPosition();



        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        //setting drive activities
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //make sure you actually don't use the encoders
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //navxMicro = opMode_.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
       // gyro = ;
//        colorSensor = opMode_.hardwareMap.get(NormalizedColorSensor.class, "color");// what a nifty program.
//        colors = colorSensor.getNormalizedColors();
        lights = opMode_.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        imu = opMode_.hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );


    }

    public RobotHardware(LinearOpMode opMode, boolean roadRunnerBool) {
        opMode_ = opMode;
        ySlidesMotorLeft = opMode_.hardwareMap.get(DcMotor.class, "ySlidesLeft");
        ySlidesMotorRight = opMode_.hardwareMap.get(DcMotor.class, "ySlidesRight");
        pivotMotor = opMode_.hardwareMap.get(DcMotor.class, "pivotMotor");
        extendMotor = opMode_.hardwareMap.get(DcMotor.class, "extendMotor");

        claw = opMode_.hardwareMap.get(Servo.class, "claw");
        clawWrist = opMode_.hardwareMap.get(Servo.class, "clawWrist");
        claw.setPosition(0.01);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

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

            opMode_.sleep(2);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode_.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) -(distance * COUNTS_PER_INCH);
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
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }
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
        opMode_.telemetry.addData("Valleyx: Heading error ", headingError);
        opMode_.telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        opMode_.telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        opMode_.telemetry.update();
    }



//    public double getHeading() {
//        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return -angles.firstAngle;
//    }
//
//    public double getHeadingRadians() {
//        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        return -angles.firstAngle;
//    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getHeadingRadians() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    public void allPower(double speedLB, double speedLF, double speedRB, double speedRF)
    {
        leftFrontDrive.setPower(speedLF);
        leftBackDrive.setPower(speedLB);
        rightFrontDrive.setPower(speedRF);
        rightBackDrive.setPower(speedRB);

    }



//    public double getHue(){
//        colorSensor.setGain(gain);
//
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        return hsvValues[0];
//    }


}

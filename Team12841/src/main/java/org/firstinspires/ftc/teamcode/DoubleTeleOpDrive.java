package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.LiftHardware;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@TeleOp(name = "DoublePersonDrive")
public class DoubleTeleOpDrive extends LinearOpMode {
    IMU imu;
    RobotHardware robothardware;
    LiftHardware lifthardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.BlueL);
        //lifthardware = new LiftHardware(robothardware);
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("lFMotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("lBMotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rFMotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rBMotor");

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        //elbow variables
        boolean elbow = gamepad2.x;
        double elbowStick = gamepad2.left_stick_y;
        int elbowPosition;
        boolean elbowStickMovement = false;
        final double elbowUpSpeed = 0.2;
        final double elbowDownSpeed = -0.1;

        //elevator variables
        boolean elevator = gamepad2.y;
        double elevatorStick = gamepad2.right_stick_y;
        int elevatorPosition;
        boolean elevatorStickMovement = false;
        final double elevatorUpSpeed = 0.2;
        final double elevatorDownSpeed = -0.1;

        //intake variables
        boolean intake = gamepad2.left_bumper;
        boolean outtake = gamepad2.back;
        boolean stopIntake = gamepad2.right_bumper;

        //B U C K E T
        boolean bucket = gamepad2.b;
        boolean bucketOpen = false;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        // BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        //BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "imu");
       /* BHI260IMU.Parameters parameters = new BHI260IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);*/

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //elbowPosition = robothardware.elbowMotor.getCurrentPosition();
            //elevatorPosition = robothardware.elevatorMotor.getCurrentPosition();


            // Read inverse IMU heading, as the IMU heading is CW positive
            //double botHeading = -imu.getAngularOrientation().firstAngle;
            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            double botHeading = -orientation.getYaw(AngleUnit.RADIANS);

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

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.update();

            //all the elbow movements
            /*if (elbow)
                lifthardware.moveElbow(100,0.5);
            else if (Math.abs(elbowStick) > 0.2)
            {
                if(elbowPosition > -5)
                {
                    robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robothardware.elbowMotor.setPower((elbowPosition > 0) ? elbowUpSpeed : elbowDownSpeed);
                    elbowStickMovement = true;
                }
            }
            else if (elbowStickMovement)
            {
                robothardware.elbowMotor.setTargetPosition(elbowPosition);
                robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowStickMovement = false;
            }

            //all the elevator movements
            if (elevator)
                lifthardware.moveElevator(10,0.5);
            else if (Math.abs(elevatorStick) > 0.2)
            {
                if(elevatorPosition > -5)
                {
                    robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robothardware.elevatorMotor.setPower((elevatorPosition > 0) ? elevatorUpSpeed : elevatorDownSpeed);
                    elevatorStickMovement = true;
                }
            }
            else if (elevatorStickMovement)
            {
                robothardware.elevatorMotor.setTargetPosition(elevatorPosition);
                robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorStickMovement = false;
            }

            //all the intake movements
            if (intake)
                robothardware.intakeMotor.setPower(0.5);
            else if (stopIntake)
                robothardware.intakeMotor.setPower(0);
            else if (outtake)
                robothardware.intakeMotor.setPower(-0.5);

            //B U C K E T
            if (bucket)
            {
                if (bucketOpen == false) {
                    robothardware.bucket.setPosition(0.5);
                    bucketOpen = true;
                }
                else {
                    robothardware.bucket.setPosition(0);
                    bucketOpen = false;
                }
            }*/
        }
    }
}

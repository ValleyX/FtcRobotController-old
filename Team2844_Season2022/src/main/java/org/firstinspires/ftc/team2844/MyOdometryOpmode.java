package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



/**
 * Created by Sarthak on 10/4/2019.
 */
@Disabled
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    public final double OD_COUNTS_PER_MOTOR_REV = 8192;    //  AndyMark Motor Encoder
    public final double OD_DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public final double OD_ONE_MOTOR_COUNT = OD_COUNTS_PER_MOTOR_REV * OD_DRIVE_GEAR_REDUCTION;
    public final double OD_Distance_in_one_rev = 2.0 * Math.PI; //in
    public final double OD_COUNTS_PER_INCH = OD_ONE_MOTOR_COUNT / OD_Distance_in_one_rev;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rightFront", rbName = "rightBack", lfName = "leftFront", lbName = "leftBack";
    String verticalLeftEncoderName = "leftEncoder", verticalRightEncoderName = "rightEncoder", horizontalEncoderName = "horizontalEncoder";

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();


        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, OD_COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
       goToPositionForward(0*OD_COUNTS_PER_INCH,-10*OD_COUNTS_PER_INCH,.5,0,1*OD_COUNTS_PER_INCH);
        //goToPositionStrafe(10*OD_COUNTS_PER_INCH,0*OD_COUNTS_PER_INCH,.5,0,1*OD_COUNTS_PER_INCH);




        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OD_COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OD_COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPositionForward(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (opModeIsActive() && distance > allowableDistanceError) {

             distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
             distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            distance = Math.hypot(distanceToX, distanceToY);


            telemetry.addData("robot movement x:", robotMovementX);
            telemetry.addData("robot movement y:", robotMovementY);
            telemetry.addData("movement angle", movementAngle);
            telemetry.addData("turn Correction:", turnCorrection);
            telemetry.addData("distance:", distance);
            telemetry.update();
            allPower(robotMovementY);

        }
        allPower(0);

    }public void goToPositionStrafe(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (opModeIsActive() && distance > allowableDistanceError) {

            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            distance = Math.hypot(distanceToX, distanceToY);


            telemetry.addData("robot movement x:", robotMovementX);
            telemetry.addData("robot movement y:", robotMovementY);
            telemetry.addData("movement angle", movementAngle);
            telemetry.addData("turn Correction:", turnCorrection);
            telemetry.addData("distance:", distance);
            telemetry.update();
            strafePower(robotMovementX);

        }
        allPower(0);

    }



    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);

        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);
;
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setDirection(DcMotor.Direction.REVERSE);
        horizontal.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    private void allPower(double speed)
    {
        right_front.setPower(-speed);
        left_back.setPower(-speed);
        left_front.setPower(-speed);
        right_back.setPower(-speed);

    }
    private void strafePower(double speed)
    {
        right_front.setPower(speed);
        left_back.setPower(speed);
        left_front.setPower(-speed);
        right_back.setPower(-speed);

    }
}

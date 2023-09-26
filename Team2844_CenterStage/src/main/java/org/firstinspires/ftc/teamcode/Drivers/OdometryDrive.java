package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.team2844.RobotHardware;

import java.io.File;

public class OdometryDrive {
    //call the robot (to be readded when ready
    private RobotHardware robot_;

    //variables needed to help run program
    private ElapsedTime runtime_;
    private boolean waiting_;
    //Thread run condition
    private boolean isRunning = true;

    //odometry framework (to be built later as well possibly renamed
    public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //creates thread
    public Thread positionThread;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    //multipliers for the encoders
    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    //OdometryGlobalCoordinatePosition globalPositionUpdate;
    //globalPositionUpdate =  new OdometryGlobalCoordinatePosition(robot_.leftVerticalEncoder, robot_.rightVerticalEncoder, robot_.horizontalEncoder, robot_.OD_COUNTS_PER_INCH, 75);

    //instatiate the method
    public OdometryDrive(RobotHardware robot ) {
        //set variables
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;

        //to be readded w/ robot hardware
        //fill variables for the global position and the thread
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot_.verticalLeft, robot_.verticalRight, robot_.horizontal, robot_.OD_COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);

        //start the thread
        positionThread.start();
    }

    //moves robot forward based of target and current position
    public void goToPositionForward(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        //multiply distances by a constant
        xTarget *= robot_.OD_COUNTS_PER_INCH;
        yTarget *= robot_.OD_COUNTS_PER_INCH;
        allowableDistanceError *= robot_.OD_COUNTS_PER_INCH;

        //calculate how close we are
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (robot_.OpMode_.opModeIsActive() && Math.abs(distanceToY) > allowableDistanceError) {

            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            distance = Math.hypot(distanceToX, distanceToY);


            robot_.OpMode_.telemetry.addData("robot movement x:", robotMovementX);
            robot_.OpMode_.telemetry.addData("robot movement y:", robotMovementY);
            robot_.OpMode_.telemetry.addData("X distance:", distanceToX);
            robot_.OpMode_.telemetry.addData("y distance:", distanceToY);
            robot_.OpMode_.telemetry.addData("movement angle", movementAngle);
            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);
            robot_.OpMode_.telemetry.addData("distance:", distance);
            robot_.OpMode_.telemetry.addData("glob x: ",globalPositionUpdate.returnXCoordinate());
            robot_.OpMode_.telemetry.addData("glob y: ",globalPositionUpdate.returnYCoordinate());
            robot_.OpMode_.telemetry.addData("glob orient: ",globalPositionUpdate.returnOrientation());
            robot_.OpMode_.telemetry.addData("encoder left: ", robot_.verticalLeft.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder right: ", robot_.verticalRight.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder horz: ", robot_.horizontal.getCurrentPosition());

            System.out.println("ValleyX: distanceToX " + distanceToX);
            System.out.println("ValleyX: distanceToY " + distanceToY);
            System.out.println("ValleyX: distance " + distance);
            System.out.println("ValleyX: turnCorrection " + turnCorrection);
            System.out.println("ValleyX: robotMovementY " + robotMovementY);
            double c_nPi = .08;

            //robot_.OpMode_.telemetry.update();
            // (don't add back) robot_.allpower(-robotMovementY);
            if (turnCorrection > 0)
            {
                robot_.leftPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
                robot_.rightPower(-robotMovementY);
            }
            else
            {
                robot_.rightPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
                robot_.leftPower(-robotMovementY);
            }
            robot_.OpMode_.idle();
        }
        robot_.allpower(0);

    }

    public void goToPositionSide(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        xTarget *= robot_.OD_COUNTS_PER_INCH;
        yTarget *= robot_.OD_COUNTS_PER_INCH;
        allowableDistanceError *= robot_.OD_COUNTS_PER_INCH;
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (robot_.OpMode_.opModeIsActive() && Math.abs(distanceToX) > allowableDistanceError) {

            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            distance = Math.hypot(distanceToX, distanceToY);


            robot_.OpMode_.telemetry.addData("robot movement x:", robotMovementX);
            robot_.OpMode_.telemetry.addData("robot movement y:", robotMovementY);
            robot_.OpMode_.telemetry.addData("X distance:", distanceToX);
            robot_.OpMode_.telemetry.addData("y distance:", distanceToY);
            robot_.OpMode_.telemetry.addData("movement angle", movementAngle);
            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);
            robot_.OpMode_.telemetry.addData("distance:", distance);
            robot_.OpMode_.telemetry.addData("glob x: ",globalPositionUpdate.returnXCoordinate());
            robot_.OpMode_.telemetry.addData("glob y: ",globalPositionUpdate.returnYCoordinate());
            robot_.OpMode_.telemetry.addData("glob orient: ",globalPositionUpdate.returnOrientation());
            robot_.OpMode_.telemetry.addData("encoder left: ", robot_.verticalLeft.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder right: ", robot_.verticalRight.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("encoder horz: ", robot_.horizontal.getCurrentPosition());

            System.out.println("ValleyX: distanceToX " + distanceToX);
            System.out.println("ValleyX: distanceToY " + distanceToY);
            System.out.println("ValleyX: distance " + distance);
            System.out.println("ValleyX: turnCorrection " + turnCorrection);
            System.out.println("ValleyX: robotMovementY " + robotMovementY);
            double c_nPi = .08;

            robot_.OpMode_.telemetry.update();
            // dont add robot_.allpower(-robotMovementY);
            // dont add   if (turnCorrection > 0)
            // dont add   {
            // dont add robot_.leftPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
            //dont add robot_.rightPower(-robotMovementY);
            robot_.motorBackLeft.setPower(robotMovementX - (c_nPi * Math.abs(turnCorrection)));
            robot_.motorFrontLeft.setPower(-robotMovementX + (c_nPi * Math.abs(turnCorrection)));
            robot_.motorBackRight.setPower(-robotMovementX + (c_nPi * Math.abs(turnCorrection)));
            robot_.motorFrontRight.setPower((robotMovementX - c_nPi * Math.abs(turnCorrection)));

            // dont add }

           /*(don't add)
           // else
          //  {
                robot_.leftBack.setPower(-robotMovementX +  (c_nPi * Math.abs(turnCorrection)));
                robot_.leftFront.setPower(robotMovementX - (c_nPi * Math.abs(turnCorrection)));
                robot_.rightBack.setPower(robotMovementX - (c_nPi * Math.abs(turnCorrection)));
                robot_.rightFront.setPower(-robotMovementX + (c_nPi * Math.abs(turnCorrection)));
               // robot_.rightPower(-robotMovementY - (c_nPi * Math.abs(turnCorrection)));
               // robot_.leftPower(-robotMovementY);
          //  }
          */



            robot_.OpMode_.idle();
        }
        robot_.allpower(0);

    }

    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


}

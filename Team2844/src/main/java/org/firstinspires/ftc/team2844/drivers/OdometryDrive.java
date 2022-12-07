

package org.firstinspires.ftc.team2844.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.team2844.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.team2844.RobotHardware;

import java.io.File;

public class OdometryDrive {
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    //Thead run condition
    private boolean isRunning = true;

    public OdometryGlobalCoordinatePosition globalPositionUpdate;
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

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    //OdometryGlobalCoordinatePosition globalPositionUpdate;
    //globalPositionUpdate =  new OdometryGlobalCoordinatePosition(robot_.leftVerticalEncoder, robot_.rightVerticalEncoder, robot_.horizontalEncoder, robot_.OD_COUNTS_PER_INCH, 75);

    public OdometryDrive(RobotHardware robot) {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot_.verticalLeft, robot_.verticalRight, robot_.horizontal, robot_.OD_COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }
    public void goToPositionForward(double xTarget, double yTarget, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        xTarget *= robot_.OD_COUNTS_PER_INCH;
        yTarget *= robot_.OD_COUNTS_PER_INCH;
        allowableDistanceError *= robot_.OD_COUNTS_PER_INCH;
        double distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
        double distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (robot_.OpMode_.opModeIsActive() && distance > allowableDistanceError) {

            distanceToX = xTarget - globalPositionUpdate.returnXCoordinate();
            distanceToY = yTarget - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(movementAngle, robotPower);
            double robotMovementY = calculateY(movementAngle, robotPower);
            double turnCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            distance = Math.hypot(distanceToX, distanceToY);


            robot_.OpMode_.telemetry.addData("robot movement x:", robotMovementX);
            robot_.OpMode_.telemetry.addData("robot movement y:", robotMovementY);
            robot_.OpMode_.telemetry.addData("movement angle", movementAngle);
            robot_.OpMode_.telemetry.addData("turn Correction:", turnCorrection);
            robot_.OpMode_.telemetry.addData("distance:", distance);
            robot_.OpMode_.telemetry.update();
            robot_.allpower(-robotMovementY);

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

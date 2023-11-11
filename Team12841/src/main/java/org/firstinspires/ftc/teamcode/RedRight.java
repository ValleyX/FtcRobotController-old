package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "RedRight")

public class RedRight extends LinearOpMode {
    RobotHardware robotHardware;
    AprilTagCamera aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.RedR);
//        Camera.SkystoneDeterminationPipeline.MarkerPos markerPosSeen;
        aprilTag = new AprilTagCamera(this, robotHardware);


        while (opModeInInit()) {
            telemetry.addData("Position", robotHardware.camera.pipeline.markerPos);
            telemetry.addData("Blue Pos 1 Color RGB ", robotHardware.camera.pipeline.avgBlue1);
            telemetry.addData("Blue Pos 2 Color RGB ", robotHardware.camera.pipeline.avgBlue2);
            telemetry.addData("Red Pos 1 Color RGB ", robotHardware.camera.pipeline.avgRed1);
            telemetry.addData("Red Pos 2 Color RGB ", robotHardware.camera.pipeline.avgRed2);//no i will keep putting weird comments in your code no
            telemetry.update();
//          markerPosSeen = robotHardware.camera.pipeline.markerPos;
        }

//        markerPosSeen = robotHardware.camera.pipeline.markerPos;
//        String testMarkerPosSeen = "Left";

        robotHardware.camera.closeCamera();
        robotHardware.imu.resetYaw();

        if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Left) {
            aprilTag.targetFound = false;
            aprilTag.Desired_Tag_Id = 4;
            aprilTag.initAprilTag();

            robotHardware.driveStraight(0.5, 28, 0.0);
            robotHardware.turnToHeading(0.5, 90);
            robotHardware.driveStraight(0.3, 8, 90);
            robotHardware.turnToHeading(0.5, 90);

            double XError = 0;

            while (opModeIsActive()) {
                aprilTag.targetFound = false;
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.aprilTag.getDetections();
                if (currentDetections != null) {
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) &&
                                ((aprilTag.Desired_Tag_Id < 0) || (detection.id == aprilTag.Desired_Tag_Id))) {
                            aprilTag.targetFound = true;
                            aprilTag.desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                        }
                    }


                    // Tell the driver what we see, and what to do.
                    if (aprilTag.targetFound) {
                        telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                        telemetry.addData("Target", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                        telemetry.addData("Range", "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                        telemetry.addData("Bearing", "%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                        telemetry.addData("Yaw", "%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
                        telemetry.addData("X", "%3.0f ", aprilTag.desiredTag.ftcPose.x);
                        telemetry.addData("Y", "%3.0f ", aprilTag.desiredTag.ftcPose.y);
                    } else {
                        telemetry.addData(">", "Drive using joysticks to find valid target\n");
                    }

                    // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                    if (aprilTag.targetFound) {

                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double rangeError = (aprilTag.desiredTag.ftcPose.range - aprilTag.DESIRED_DISTANCE);
                        double headingError = aprilTag.desiredTag.ftcPose.bearing;
                        double yawError = aprilTag.desiredTag.ftcPose.yaw;
                        XError = aprilTag.desiredTag.ftcPose.x;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        aprilTag.drive = Range.clip(rangeError * aprilTag.SPEED_GAIN, -aprilTag.MAX_AUTO_SPEED, aprilTag.MAX_AUTO_SPEED);
//                aprilTag.turn = Range.clip(headingError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
                        aprilTag.turn = Range.clip(yawError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
//                aprilTag.strafe = Range.clip(-yawError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);
                        aprilTag.strafe = Range.clip(-XError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    } else {

                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                        aprilTag.drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                        aprilTag.strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                        aprilTag.turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    }
                    telemetry.addData("imu yaw", robotHardware.getHeading());
                    telemetry.update();

                    // Apply desired axes motions to the drivetrain.
                    //aprilTag.moveRobot(aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    //aprilTag.moveRobot(aprilTag.drive, -aprilTag.turn, 0);
                    //if (Math.abs(aprilTag.turn) < 1)
                    //    aprilTag.turn = 0;

                    aprilTag.moveRobot(0, aprilTag.strafe, 0);
                    if (Math.abs(robotHardware.getHeading() - 90) > 1) {
                        robotHardware.turnToHeading(0.25, -90);
                    }
                    if (Math.abs(XError) < 0.5) {
                        robotHardware.driveStraight(0.5, aprilTag.desiredTag.ftcPose.range - 6, -90.0);
                        break;
                    }
                    sleep(10);
                }
            }


        } else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Center) {
            aprilTag.targetFound = false;
            aprilTag.Desired_Tag_Id = 5;
            aprilTag.initAprilTag();

            robotHardware.driveStraight(0.5, -28, 0.0);
            robotHardware.turnToHeading(0.5, -90);
            robotHardware.driveStraight(0.3, -8, -90);
            robotHardware.turnToHeading(0.5, -90);

            double XError = 0;

            while (opModeIsActive()) {
                aprilTag.targetFound = false;
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.aprilTag.getDetections();
                if (currentDetections != null) {
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) &&
                                ((aprilTag.Desired_Tag_Id < 0) || (detection.id == aprilTag.Desired_Tag_Id))) {
                            aprilTag.targetFound = true;
                            aprilTag.desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                        }
                    }


                    // Tell the driver what we see, and what to do.
                    if (aprilTag.targetFound) {
                        telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                        telemetry.addData("Target", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                        telemetry.addData("Range", "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                        telemetry.addData("Bearing", "%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                        telemetry.addData("Yaw", "%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
                        telemetry.addData("X", "%3.0f ", aprilTag.desiredTag.ftcPose.x);
                        telemetry.addData("Y", "%3.0f ", aprilTag.desiredTag.ftcPose.y);
                    } else {
                        telemetry.addData(">", "Drive using joysticks to find valid target\n");
                    }

                    // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                    if (aprilTag.targetFound) {

                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double rangeError = (aprilTag.desiredTag.ftcPose.range - aprilTag.DESIRED_DISTANCE);
                        double headingError = aprilTag.desiredTag.ftcPose.bearing;
                        double yawError = aprilTag.desiredTag.ftcPose.yaw;
                        XError = aprilTag.desiredTag.ftcPose.x;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        aprilTag.drive = Range.clip(rangeError * aprilTag.SPEED_GAIN, -aprilTag.MAX_AUTO_SPEED, aprilTag.MAX_AUTO_SPEED);
//                aprilTag.turn = Range.clip(headingError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
                        aprilTag.turn = Range.clip(yawError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
//                aprilTag.strafe = Range.clip(-yawError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);
                        aprilTag.strafe = Range.clip(-XError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    } else {

                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                        aprilTag.drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                        aprilTag.strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                        aprilTag.turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    }
                    telemetry.addData("imu yaw", robotHardware.getHeading());
                    telemetry.update();

                    // Apply desired axes motions to the drivetrain.
                    //aprilTag.moveRobot(aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    //aprilTag.moveRobot(aprilTag.drive, -aprilTag.turn, 0);
                    //if (Math.abs(aprilTag.turn) < 1)
                    //    aprilTag.turn = 0;

                    aprilTag.moveRobot(0, aprilTag.strafe, 0);
                    if (Math.abs(robotHardware.getHeading() - 90) > 1) {
                        robotHardware.turnToHeading(0.25, -90);
                    }
                    if (Math.abs(XError) < 0.5) {
                        robotHardware.driveStraight(0.5, aprilTag.desiredTag.ftcPose.range - 6, -90.0);
                        break;
                    }
                    sleep(10);
                }
            }


        } else if (robotHardware.camera.pipeline.markerPos == Camera.SkystoneDeterminationPipeline.MarkerPos.Right) {
            aprilTag.targetFound = false;
            aprilTag.Desired_Tag_Id = 6;
            aprilTag.initAprilTag();

            robotHardware.driveStraight(0.5, -28, 0.0);
            robotHardware.turnToHeading(0.5, -90);
            robotHardware.driveStraight(0.3, -8, -90);
            robotHardware.turnToHeading(0.5, -90);

            double XError = 0;

            while (opModeIsActive()) {
                aprilTag.targetFound = false;
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.aprilTag.getDetections();
                if (currentDetections != null) {
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) &&
                                ((aprilTag.Desired_Tag_Id < 0) || (detection.id == aprilTag.Desired_Tag_Id))) {
                            aprilTag.targetFound = true;
                            aprilTag.desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                        }
                    }


                    // Tell the driver what we see, and what to do.
                    if (aprilTag.targetFound) {
                        telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                        telemetry.addData("Target", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                        telemetry.addData("Range", "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                        telemetry.addData("Bearing", "%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                        telemetry.addData("Yaw", "%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
                        telemetry.addData("X", "%3.0f ", aprilTag.desiredTag.ftcPose.x);
                        telemetry.addData("Y", "%3.0f ", aprilTag.desiredTag.ftcPose.y);
                    } else {
                        telemetry.addData(">", "Drive using joysticks to find valid target\n");
                    }

                    // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                    if (aprilTag.targetFound) {

                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double rangeError = (aprilTag.desiredTag.ftcPose.range - aprilTag.DESIRED_DISTANCE);
                        double headingError = aprilTag.desiredTag.ftcPose.bearing;
                        double yawError = aprilTag.desiredTag.ftcPose.yaw;
                        XError = aprilTag.desiredTag.ftcPose.x;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        aprilTag.drive = Range.clip(rangeError * aprilTag.SPEED_GAIN, -aprilTag.MAX_AUTO_SPEED, aprilTag.MAX_AUTO_SPEED);
//                aprilTag.turn = Range.clip(headingError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
                        aprilTag.turn = Range.clip(yawError * aprilTag.TURN_GAIN, -aprilTag.MAX_AUTO_TURN, aprilTag.MAX_AUTO_TURN);
//                aprilTag.strafe = Range.clip(-yawError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);
                        aprilTag.strafe = Range.clip(-XError * aprilTag.STRAFE_GAIN, -aprilTag.MAX_AUTO_STRAFE, aprilTag.MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    } else {

                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                        aprilTag.drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                        aprilTag.strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                        aprilTag.turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    }
                    telemetry.addData("imu yaw", robotHardware.getHeading());
                    telemetry.update();

                    // Apply desired axes motions to the drivetrain.
                    //aprilTag.moveRobot(aprilTag.drive, aprilTag.strafe, aprilTag.turn);
                    //aprilTag.moveRobot(aprilTag.drive, -aprilTag.turn, 0);
                    //if (Math.abs(aprilTag.turn) < 1)
                    //    aprilTag.turn = 0;

                    aprilTag.moveRobot(0, aprilTag.strafe, 0);
                    if (Math.abs(robotHardware.getHeading() - 90) > 1) {
                        robotHardware.turnToHeading(0.25, -90);
                    }
                    if (Math.abs(XError) < 0.5) {
                        robotHardware.driveStraight(0.5, aprilTag.desiredTag.ftcPose.range - 6, -90.0);
                        break;
                    }
                    sleep(10);
                }
            }
        }
    }
}
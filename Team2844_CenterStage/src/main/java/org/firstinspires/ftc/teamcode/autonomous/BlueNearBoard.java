package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivers.AprilTag;
import org.firstinspires.ftc.teamcode.Drivers.GyroDrive;
import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;

import org.firstinspires.ftc.teamcode.Drivers.OdometryDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//robot must pe put in the same place to relatively same position per game to be consistant
//Position distance from corner near board is about 48 inches
@Autonomous(name="AutoBlueNearBoard")
public class BlueNearBoard extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //Using test hardware for now because test hardware has properties needed for practice
        RobotHardware robot = new RobotHardware(this,true); //checkBlue is true to only find
        GyroDrive gyroDrive = new GyroDrive(robot);
        RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        AprilTagDetection desiredTag = null;
        AprilTag aprilTag;

        int AprilTagID = 2;
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        double distanceToBoard = 0;
        double startingDistanceFromBoard = 49;
/*
        sleep(2000); // play with tested value
        robot.switchableWebcam.setPipeline(null); // Turning off camera
        robot.switchableWebcam.closeCameraDevice();
        sleep(2000); // play with tested value

        RobotHardware.AprilTag aprilTag = new RobotHardware.AprilTag(robot);


        waitForStart();
*/
        /*
        while(opModeInInit()){
            //to tell user what values the camera sees
            telemetry.addData("r value", robot.pipeline.avgR);
            telemetry.addData("b value", robot.pipeline.avgB);
            telemetry.addData("r2 value", robot.pipeline.avg2R);
            telemetry.addData("b2 value", robot.pipeline.avg2B);
            telemetry.addData("r3 value", robot.pipeline.avg3R);
            telemetry.addData("b3 value", robot.pipeline.avg3B);

            telemetry.addData("teamProp position", robot.pipeline.position);

            telemetry.update();

            position = robot.pipeline.position;


        }

        robot.switchableWebcam.setPipeline(null); // Turning off camera
        robot.switchableWebcam.closeCameraDevice();
        sleep(2000); // play with tested value
*/
       // RobotHardware.AprilTag aprilTag = new RobotHardware.AprilTag(robot);
        aprilTag = new AprilTag(robot);
        aprilTag.initAprilTag(robot);
        //aprilTag.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        aprilTag.setManualExposure(robot,6,350);

        waitForStart();
        robot.imu.resetYaw();


       // desiredTag = aprilTag.aprilTagDetected(AprilTagID);


     //   robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Making motors run off the encoders
    //    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     //   robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     //   robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



     //   gyroDrive.driveStraight(0.5,24,0);
     //   sleep(100);
   //     gyroDrive.turnToHeading(0.5,-90);
    //    sleep(1000);


        double rangeError;
        double headingError;
        double yawError;
        //desiredTag.ftcPose = new AprilTagPoseFtc(3,3);
        Boolean found = false;
        Boolean targetReached = false;

        while (opModeIsActive() && !targetReached){
           // if(aprilTag.isAprilTagDetected(AprilTagID)){
                if ((desiredTag = aprilTag.aprilTagDetected(robot,AprilTagID)) != null)
                {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    rangeError = (desiredTag.ftcPose.range - robot.DESIRED_DISTANCE);
                    headingError = desiredTag.ftcPose.bearing;
                    yawError = desiredTag.ftcPose.yaw;


                    drive = Range.clip(rangeError * robot.SPEED_GAIN, -robot.MAX_AUTO_SPEED, robot.MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * robot.TURN_GAIN, -robot.MAX_AUTO_TURN, robot.MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * robot.STRAFE_GAIN, -robot.MAX_AUTO_STRAFE, robot.MAX_AUTO_STRAFE);



                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                   // drive = Range.clip(rangeError * robot.SPEED_GAIN, -robot.MAX_AUTO_SPEED, robot.MAX_AUTO_SPEED);
                 //   turn = Range.clip(headingError * robot.TURN_GAIN, -robot.MAX_AUTO_TURN, robot.MAX_AUTO_TURN);
                   // strafe = Range.clip(-yawError * robot.STRAFE_GAIN, -robot.MAX_AUTO_STRAFE, robot.MAX_AUTO_STRAFE);

                    //robot.moveRobot(drive, strafe, turn);
                    found = true;


                    //robot.moveRobot(0,strafe,0);
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f rangeError %5.2f", drive, strafe, turn, rangeError);
                    System.out.printf("ValleyX: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    telemetry.update();

                    //if (Math.abs(rangeError) < 0.4) //was 0.5
                    //    break;


                    distanceToBoard += drive;
                    robot.moveRobot(drive, strafe, turn);

                //}
            }
            else{ //not in view
                //robot.moveRobot(0,0,0); //stops robot from moving
              //  if(found){
                //    continue;
                //}


                /*
                if (distanceToBoard < startingDistanceFromBoard) {
                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Making motors run off the encoders
                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    gyroDrive.driveStraight(0.2, 0.5, -90);
                    distanceToBoard += 0.5;

                }

                 */





            }

          //  robot.moveRobot(drive, strafe, turn);
           sleep(10);

        }






    }


}

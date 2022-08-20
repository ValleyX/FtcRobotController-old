package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.LiftDriverTest;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

//@Autonomous(name="RedWarehouseSide")
@Autonomous(name="RedWarehouseSide")
public class RedWarehouseSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 320, 340, RobotHardware.cameraSelection.UP);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);


        /** directions
         1. variables
         2. setting a timer at the start of auto. then moving up to grab team element using the camera to sense
         where the team element is.
         3. turns to go place preloaded block onto the specific layer
         4. backs up (to not hit the wobbles), turns towards warehouse, lowers the lift to be less front heavy,
         and drives into the warehouse. the robot sets lift all the way down to allow the camera to sense the
         yellow block. then turns the yellow block scanner pipeline on




         */

        // 1.
        double dist;
        double motorspeed = 0.2;

        double armdown = 0.88;
        double armup = 0.2;
        double grabin = 0.9;
        double grabout = 0;


        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;
        //Phone outputs
        while (!isStarted()) {
            path = robot.pipeline.position;
            telemetry.addData("AverageMiddle", robot.pipeline.SkystoneAverageMiddle);
            telemetry.addData("AverageLeft", robot.pipeline.SkystoneAverageLeft);
            telemetry.addData("AverageRight", robot.pipeline.SkystoneAverageRight);
            telemetry.addData("Max avg", Math.max(Math.max(robot.pipeline.SkystoneAverageMiddle, robot.pipeline.SkystoneAverageLeft), robot.pipeline.SkystoneAverageRight));
            telemetry.addData("Position", path);
            telemetry.update();
            robot.grab.setPosition(grabout);
            sleep(500);
            robot.arm.setPosition(0.3);

        }

        // 2.

        ElapsedTime elapsedTimestart = new ElapsedTime();
        elapsedTimestart.reset();

        robot.arm.setPosition(armdown);

        headingdrive.gyroDrive(1, 8, 0);

        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Middle) {
            headingdrive.gyroDrive(0.2, 7, 0);
            robot.grab.setPosition(grabin);
            sleep(500);
            robot.arm.setPosition(armup);
        } else {
            headingdrive.gyroDrive(1, 20, 0);
            headingdrive.gyroDrive(1, -13, 0);
            robot.arm.setPosition(armup);

        }


        // 3.
        headingdrive.gyroTurn(0.7, -35); //-15


        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Left) {
            dist = 5;
            liftto.LiftToDistance(0.9, dist, false);
        } else if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Middle) {
            dist = 11;
            liftto.LiftToDistance(0.9, dist, false);
        } else {
            dist = 17;
            liftto.LiftToDistance(0.9, dist, false);
        }

        headingdrive.gyroDrive(0.3, 14, -35);

        // test code go to lift motor to understand what is happening
        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // test code go to lift motor to understand what is happening


        robot.superintake.setPower(-1); // -1 shoots block out
        sleep(500);
        robot.superintake.setPower(0); // 1 sucks block in

        // 4.

        headingdrive.gyroDrive(1, -6, -20);

        liftto.LiftToDistance(1, -dist + 5, false);

        headingdrive.gyroTurn(0.8, 87);

        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Robot going into warehouse
        headingdrive.gyroDrive(1, 52, 90);


        //this ensures that the lift doesnt go down on a block or ball
        headingdrive.gyroDrive(1, -2, 90);

        liftto.LiftToDistance(0.8, -dist, true);

        headingdrive.gyroTurn(1, 120);


        robot.switchableWebcam.setPipeline(robot.goldPipeline);


        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        double heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (robot.goldPipeline.isFound() && elapsedTime.seconds() <= 1) {
            if (robot.goldPipeline.getXPosition() > 350) {
                robot.leftBack.setPower(motorspeed);
                robot.leftFront.setPower(motorspeed);
                robot.rightBack.setPower(-motorspeed);
                robot.rightFront.setPower(-motorspeed);
            } else if (robot.goldPipeline.getXPosition() < 290) {
                robot.leftBack.setPower(-motorspeed);
                robot.leftFront.setPower(-motorspeed);
                robot.rightBack.setPower(motorspeed);
                robot.rightFront.setPower(motorspeed);

            } else {
                robot.leftBack.setPower(0);
                robot.leftFront.setPower(0);
                robot.rightBack.setPower(0);
                robot.rightFront.setPower(0);
            }

            //talking to the phone about positions of the boxes the robot sees
            telemetry.addData("Is Found", robot.goldPipeline.isFound());

            if (robot.goldPipeline.isFound()) {
                telemetry.addData("Aligned", robot.goldPipeline.getAligned());
                telemetry.addData("Get X", robot.goldPipeline.getXPosition());
                telemetry.addData("width", robot.goldPipeline.getFoundRect().width);
            }
            telemetry.update();
        }

        //Setting variables
        int LeftFrontEncoder = robot.leftFront.getCurrentPosition();
        int LeftBackEncoder = robot.leftBack.getCurrentPosition();
        int RightFrontEncoder = robot.rightFront.getCurrentPosition();
        int RightBackEncoder = robot.rightBack.getCurrentPosition();

        heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //      robot.allpower(0.4);
        //  robot.superintake.setPower(0.5);
        // headingdrive.gyroDrive(.5,4,heading);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while ((robot.blocksensor.getDistance(DistanceUnit.CM) > 4.5) && (runtime.seconds() < 2)) {
            robot.allpower(0.25);
            robot.superintake.setPower(1);
            telemetry.addData("distance from block", robot.blocksensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        //Robot stopping and starting intake for box
        robot.allpower(0);

        headingdrive.gyroTurn(1,0);

        robot.allpower(0);


    }

}
        /*
        robot.superintake.setPower(0.2);
        robot.switchableWebcam.setActiveCamera(robot.WebcamUp);


        //Definitions for Robot finding the Targeted position
        robot.leftFront.setTargetPosition(LeftFrontEncoder);
        robot.leftBack.setTargetPosition(LeftBackEncoder);
        robot.rightBack.setTargetPosition(RightBackEncoder);
        robot.rightFront.setTargetPosition(RightFrontEncoder);

        //Defintions for Robot motors moving toward target
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double speed = 0.20;
        // reset the timeout time and start motion.
        robot.leftFront.setPower(Math.abs(speed));
        robot.rightFront.setPower(Math.abs(speed));
        robot.leftBack.setPower(Math.abs(speed));
        robot.rightBack.setPower(Math.abs(speed));

        while(robot.leftFront.isBusy() || robot.rightFront.isBusy()
                || robot.leftBack.isBusy() || robot.rightBack.isBusy());

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.allpower(0);


        if (elapsedTimestart.seconds() <= 20) {
            //Having the Robot move out of the box
            robot.switchableWebcam.setPipeline(robot.redPipeline);

            headingdrive.gyroTurn(1, -90);

            liftto.LiftToDistance(1, 5, false);

            headingdrive.gyroDrive(1, -10, -90);

            elapsedTime.reset();
            while (robot.OpMode_.opModeIsActive() &&
                    (robot.liftmotor.isBusy()) && (robot_.liftdowntouch.getState() == true )
                    && (elapsedTime.seconds() < 1));

            headingdrive.gyroDrive(1, 50, -90);

            headingdrive.gyroTurn(1, -25);

            liftto.LiftToDistance(1,7,false);


            robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.BLACK, RevBlinkinLedDriver.BlinkinPattern.BLACK);


            //alignment to red post
            double width; // = 200
            elapsedTime.reset();


                heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX heading: " + heading);
                if (robot.redPipeline.isFound()) {
                    width = robot.redPipeline.getFoundRect().width;
                    System.out.println(("ValleyX X = " + robot.redPipeline.getXPosition()));

                    if (robot.redPipeline.getXPosition() > 330 && heading <= -30) {
                        robot.leftBack.setPower(motorspeed);
                        robot.leftFront.setPower(motorspeed);
                        robot.rightBack.setPower(-motorspeed);
                        robot.rightFront.setPower(-motorspeed);
                    } else if (robot.redPipeline.getXPosition() < 310 && heading >= -60) {
                        robot.leftBack.setPower(-motorspeed);
                        robot.leftFront.setPower(-motorspeed);
                        robot.rightBack.setPower(motorspeed);
                        robot.rightFront.setPower(motorspeed);
                    } else {
                        motorspeed = 0;
                        robot.leftBack.setPower(-motorspeed);
                        robot.leftFront.setPower(-motorspeed);
                        robot.rightBack.setPower(motorspeed);
                        robot.rightFront.setPower(motorspeed);
                    }
                    telemetry.addData("Aligned", robot.redPipeline.getAligned());
                    telemetry.addData("Get X", robot.redPipeline.getXPosition());
                    telemetry.addData("width", robot.redPipeline.getFoundRect().width);
                    telemetry.update();
            }

            elapsedTime.reset();
            while (robot.OpMode_.opModeIsActive() &&
                    (robot.liftmotor.isBusy()) && (robot_.liftdowntouch.getState() == true )
                    && (elapsedTime.seconds() < 1));

            elapsedTime.reset();

            while (robot.redPipeline.getFoundRect().width <= 23 )
            {
                    robot.allpower(0.05);
            }

            robot.allpower(0);



        System.out.println("ValleyX Width: " + robot.redPipeline.getFoundRect().width);
        while (isStarted()) {
            telemetry.addData("Is Found", robot.redPipeline.isFound());
            telemetry.addData("redTheshold", robot.redPipeline.redTheshold);

            if (robot.redPipeline.isFound()) {
                telemetry.addData("Aligned", robot.redPipeline.getAligned());
                telemetry.addData("Get X", robot.redPipeline.getXPosition());
                telemetry.addData("width", robot.redPipeline.getFoundRect().width);
            }
            telemetry.update();
        }


            heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            headingdrive.gyroTurn(0.6,heading - 10);
            //double initalWidth = robot.redPipeline.getFoundRect().width;
            double currentWidth = robot.redPipeline.getFoundRect().width;
            //49 is about 12inches

            //while (robot.redPipeline.getFoundRect().width <)

            if (currentWidth >= 40 && currentWidth <= 50) {

                liftto.LiftToDistance(1, 5, true);
                headingdrive.gyroDrive(0.3, 9.5, heading);
            }

            else if (currentWidth > 50) {
                liftto.LiftToDistance(1, 5, true);
                headingdrive.gyroDrive(0.3, 2, heading);
            }

            else if (currentWidth < 40 && currentWidth >= 30) {
                liftto.LiftToDistance(1, 5, true);
                headingdrive.gyroDrive(0.3, 16, heading);

            }

            else {
                liftto.LiftToDistance(1, 5, true);
                headingdrive.gyroDrive(0.3, 18, heading);

            }



            robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE, RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);



        /*-------
        robot.allpower(0.15);
        while ((currentWidth < 60)  && (elapsedTime.seconds() < 1.5)) { //94
            if (robot.redPipeline.getFoundRect().width < currentWidth) {
                //initalWidth = robot.redPipeline.getFoundRect().width;
                currentWidth = robot.redPipeline.getFoundRect().width;

                System.out.println("ValleyX Current Width: " + currentWidth);
            }
            System.out.println("ValleyX Actual Width: " + robot.redPipeline.getFoundRect().width);
            idle();

        }
        //Robot moving to warehouse
        ----

        robot.allpower(0);
        liftto.LiftToDistance(1,5, true);

        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);



            robot.superintake.setPower(-1);
            sleep(500);
            robot.superintake.setPower(0);

            heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            headingdrive.gyroDrive(1, -6, heading);     //robot moving back into warehouse

            //test
            // Stop all motion;
            robot.liftmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //test

            headingdrive.gyroTurn(1, -90);

            headingdrive.gyroDrive(1, -55, -90);

            liftto.LiftToDistance(0.8 , -5, true);

        }

        else {
             robot.allpower(0);
        }
*/












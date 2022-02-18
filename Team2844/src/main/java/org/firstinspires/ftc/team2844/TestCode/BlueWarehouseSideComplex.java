package org.firstinspires.ftc.team2844.TestCode;
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

@Autonomous(name="BlueWarehouseSideComplex")
public class BlueWarehouseSideComplex extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 320, 240 , RobotHardware.cameraSelection.DOWN);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);
        double dist;
        double motorspeed = 0.2;


        double armdown = 0.9;
        double armup= 0.2;
        double grabin = 0.9;
        double grabout= 0;



        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;
        //Phone outputs
        while (!isStarted())
        {

            robot.arm.setPosition(armdown);
            sleep(1000);
            robot.grab.setPosition(grabout);

            path = robot.pipeline.position;
            telemetry.addData("AverageMiddle", robot.pipeline.SkystoneAverageMiddle);
            telemetry.addData("AverageLeft", robot.pipeline.SkystoneAverageLeft);
            telemetry.addData("AverageRight", robot.pipeline.SkystoneAverageRight);
            telemetry.addData("Max avg", Math.max(Math.max(robot.pipeline.SkystoneAverageMiddle, robot.pipeline.SkystoneAverageLeft), robot.pipeline.SkystoneAverageRight));
            telemetry.addData("Position", path);
            telemetry.update();
        }

        //Robot moving to the wabble structure from start
        //headingdrive.gyroDrive(0.7, 30, 0);

        headingdrive.gyroDrive(1,8,0);
        headingdrive.gyroDrive(0.1,7,0 );

        robot.grab.setPosition(grabin);
        sleep(500);
        robot.arm.setPosition(armup);

        headingdrive.gyroTurn(0.7,35); //-15


        //if Challenge Box or blue box at this height lift goes
        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Left){
            dist = 5;
            liftto.LiftToDistance(0.9, dist, false);
        }

        else if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Middle) {
            dist = 11;
            liftto.LiftToDistance(0.9, dist, false);
        }

        else {
            dist = 17;
            liftto.LiftToDistance(0.9, dist, false);
        }

        headingdrive.gyroDrive(0.3,14,35);

        // check if the lift is at the right place
        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // check if the lift is at the right place


        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);

        headingdrive.gyroDrive(0.5,-6, -20);

        //liftto.LiftToDistance(0.3, -dist + 6);
        liftto.LiftToDistance(0.5,-dist+5, false );


        headingdrive.gyroTurn(0.5,-87);

        // check if the lift is at the right place
        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // check if the lift is at the right place

        //Robot going into warehouse
        headingdrive.gyroDrive(1,52,-90);

        headingdrive.gyroDrive(0.8,-5,-90);

        liftto.LiftToDistance(0.8,-dist, true );

        headingdrive.gyroTurn(0.4,-120);
        //Robot seeing gold or yellow for boxes
        robot.switchableWebcam.setPipeline(robot.goldPipeline);


        //Robot turning to boxes
        double heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //Robot finding the boxes
        while (opModeIsActive()) {
            if (robot.goldPipeline.isFound()) {
                if (robot.goldPipeline.getXPosition() > 340) {
                    robot.leftBack.setPower(motorspeed);
                    robot.leftFront.setPower(motorspeed);
                    robot.rightBack.setPower(-motorspeed);
                    robot.rightFront.setPower(-motorspeed);
                } else if (robot.goldPipeline.getXPosition() < 300) {
                    robot.leftBack.setPower(-motorspeed);
                    robot.leftFront.setPower(-motorspeed);
                    robot.rightBack.setPower(motorspeed);
                    robot.rightFront.setPower(motorspeed);
                } else {
                    break;
                }
            }
            else //not found go back to original pos
            {
                //headingdrive.gyroTurn(0.5, heading);
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

        while ((robot.blocksensor.getDistance(DistanceUnit.CM) > 4.5  ) && (runtime.seconds() < 1.5)) {
            robot.allpower(0.25);
            robot.superintake.setPower(0.8 );
            telemetry.addData("distance from block",robot.blocksensor.getDistance(DistanceUnit.CM) );
            telemetry.update();
        }
        //Robot stopping and starting intake for box
        robot.allpower(0);
        robot.superintake.setPower(0.1);
        //Robot start seeing red
        robot.switchableWebcam.setPipeline(robot.bluePipeline);

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


        double speed = 0.3;
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
        //Having the Robot move out of the box
        headingdrive.gyroTurn(1, 90);
        liftto.LiftToDistance(1,5, true);

        headingdrive.gyroDrive(1,-10,90);

        headingdrive.gyroDrive(1,50, 90);

        headingdrive.gyroTurn(1,45);

        liftto.LiftToDistance(1,-5, true);

        sleep(500);



        //alignment to red post
        double width = 200;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        //Going to Red wabble with certain speeds and looking for the red pipe
        while (opModeIsActive() /*&& (width >= 120 || width < 20)*/ && elapsedTime.seconds() <= 3) {
            heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            System.out.println("ValleyX heading: " + heading);
            if (robot.bluePipeline.isFound()) {
                width = robot.bluePipeline.getFoundRect().width;
                System.out.println(("ValleyX X = " + robot.bluePipeline.getXPosition() ));
                //telemetry.add()
                if (robot.bluePipeline.getXPosition() > 330 && heading <=-30) {
                    robot.leftBack.setPower(motorspeed);
                    robot.leftFront.setPower(motorspeed);
                    robot.rightBack.setPower(-motorspeed);
                    robot.rightFront.setPower(-motorspeed);
                } else if (robot.bluePipeline.getXPosition() < 310 && heading >= -60  ) {
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
                    break;
                }
                telemetry.addData("Aligned", robot.bluePipeline.getAligned());
                telemetry.addData("Get X", robot.bluePipeline.getXPosition());
                telemetry.addData("width", robot.bluePipeline.getFoundRect().width);
                telemetry.update();

            }
        }
/*
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

 */

        sleep(1500);
        //double initalWidth = robot.redPipeline.getFoundRect().width;
        double currentWidth = robot.bluePipeline.getFoundRect().width;
        //49 is about 12inches



        if (currentWidth >= 40 && currentWidth <= 50){
            liftto.LiftToDistance(1,5,true);
            headingdrive.gyroDrive(0.3,9.5, heading);
        }

        else if (currentWidth > 50) {
            liftto.LiftToDistance(1,5,true);
            headingdrive.gyroDrive(0.3,4,heading);
        }

        else if (currentWidth <40 && currentWidth >= 30) {
            liftto.LiftToDistance(1,5,true);
            headingdrive.gyroDrive(0.3,16,heading);

        }

        else {
            liftto.LiftToDistance(1,5,true);
            headingdrive.gyroDrive(0.3,18,heading);

        }




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

         */

        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);

        headingdrive.gyroDrive(1,-6, heading);

        headingdrive.gyroTurn(0.5,-90);

        headingdrive.gyroDrive(1,55,-90);

        liftto.LiftToDistance(0.8,-5, true);









    }

    /*
        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;

        while (!isStarted())
        {
            path = robot.pipeline.position;
            //telemetry.addData("AverageMiddle", robot.pipeline.SkystoneAverageMiddle);
            //telemetry.addData("AverageLeft", robot.pipeline.SkystoneAverageLeft);
            //telemetry.addData("AverageRight", robot.pipeline.SkystoneAverageRight);
            //telemetry.addData("Max avg", Math.max(Math.max(robot.pipeline.SkystoneAverageMiddle, robot.pipeline.SkystoneAverageLeft), robot.pipeline.SkystoneAverageRight));
            telemetry.addData("Position", path);
            telemetry.update();
        }
 IT DRIFTS
        headingdrive.gyroDrive(0.7, 8, 0);

        robot.StraifRight(0.4);
        sleep(500);

        headingdrive.gyroTurn(0.75,15);

        headingdrive.gyroDrive(1,14.8,20);




        headingdrive.gyroDrive(0.5, 30, 0);

        headingdrive.gyroDrive(0.7, -15,0);

        headingdrive.gyroTurn(0.5, 40);

        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Left){
            dist = 5;
            liftto.LiftToDistance(0.9, dist, true);
        }

        else if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Middle) {
            dist = 11;
            liftto.LiftToDistance(0.9, dist, true);
        }

        else {
            dist = 17;
            liftto.LiftToDistance(0.9, dist, true);
        }


        headingdrive.gyroDrive(0.7,12.4,40);

        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);

        headingdrive.gyroDrive(0.5,-9, 20);

        //liftto.LiftToDistance(0.3, -dist + 6);

        headingdrive.gyroTurn(0.75,-90);

        headingdrive.gyroDrive(1,60,-90);

        headingdrive.gyroTurn(0.4,0);

        liftto.LiftToDistance(0.4,-dist, true);


        /*
        sleep(1000);
        headingdrive.gyroDrive(1,-4,40);
        sleep(500);
        headingdrive.gyroTurn(0.75,-90);
        sleep(1000);
        headingdrive.gyroDrive(1,5,-90);
        sleep(1000);
        headingdrive.gyroDrive(1,30,-90);        //Going into Warehouse and correcting angle
        sleep(1000);
        headingdrive.gyroTurn(1,0);
        sleep(1000);




         */




    }


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

@Autonomous(name="BlueWarehouseSide")
public class BlueWarehouseSide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 320, 340, RobotHardware.cameraSelection.UP);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);
        double dist;
        double motorspeed = 0.2;


        double armdown = 0.89;
        double armup = 0.2;
        double grabin = 0.9;
        double grabout = 0;


        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;
        //Phone outputs
        while (!isStarted()) {

            robot.arm.setPosition(0.3);
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

        robot.arm.setPosition(armdown);

        headingdrive.gyroDrive(1, 8, 0);
        headingdrive.gyroDrive(0.1, 7, 0);

        robot.grab.setPosition(grabin);
        sleep(500);
        robot.arm.setPosition(armup);

        headingdrive.gyroTurn(0.7, 35); //-15


        //if Challenge Box or blue box at this height lift goes
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

        headingdrive.gyroDrive(0.3, 14, 35);

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

        headingdrive.gyroDrive(0.5, -6, -20);

        //liftto.LiftToDistance(0.3, -dist + 6);
        liftto.LiftToDistance(0.5, -dist + 5, false);


        headingdrive.gyroTurn(0.5, -87);

        // check if the lift is at the right place
        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // check if the lift is at the right place

        //Robot going into warehouse
        headingdrive.gyroDrive(1, 52, -90);

        headingdrive.gyroDrive(0.8, -5, -90);

        liftto.LiftToDistance(0.8, -dist, true);

        headingdrive.gyroTurn(0.4, -120);
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
            } else //not found go back to original pos
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

        while ((robot.blocksensor.getDistance(DistanceUnit.CM) > 4.5) && (runtime.seconds() < 1.5)) {
            robot.allpower(0.25);
            robot.superintake.setPower(0.8);
            telemetry.addData("distance from block", robot.blocksensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        //Robot stopping and starting intake for box
        robot.superintake.setPower(0);
        robot.allpower(0);

        headingdrive.gyroTurn(1, 0);

        robot.allpower(0);
        sleep(1000);

    }
}

package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.LiftDriverTest;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@Autonomous(name="aligntest")
public class AligningDriverTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 145, 120, RobotHardware.cameraSelection.DOWN);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);

        double motorspeed = 0.45;

        sleep(1000);

        robot.switchableWebcam.setPipeline(robot.goldPipeline);

        while (!isStarted()) {
            telemetry.addData("Is Found", robot.goldPipeline.isFound());

            if (robot.goldPipeline.isFound()) {
                telemetry.addData("Aligned", robot.goldPipeline.getAligned());
                telemetry.addData("Get X", robot.goldPipeline.getXPosition());
                telemetry.addData("width", robot.goldPipeline.getFoundRect().width);
            }
            telemetry.update();
        }


        headingdrive.gyroDrive(0.7, 30, 0);

        headingdrive.gyroDrive(0.4,-10,0);

        liftto.LiftToDistance(1, 5,true);

        headingdrive.gyroTurn(0.8, 90);

       headingdrive.gyroDrive(1,45, 90);

        liftto.LiftToDistance(1, -5,true);

        headingdrive.gyroTurn(0.7, 120);

        while (opModeIsActive()) {

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

            telemetry.addData("Is Found", robot.goldPipeline.isFound());

            if (robot.goldPipeline.isFound()) {
                telemetry.addData("Aligned", robot.goldPipeline.getAligned());
                telemetry.addData("Get X", robot.goldPipeline.getXPosition());
                telemetry.addData("width", robot.goldPipeline.getFoundRect().width);
            }
            telemetry.update();
        }


        int LeftFrontEncoder = robot.leftFront.getCurrentPosition();
        int LeftBackEncoder = robot.leftBack.getCurrentPosition();
        int RightFrontEncoder = robot.rightFront.getCurrentPosition();
        int RightBackEncoder = robot.rightBack.getCurrentPosition();



        while (robot.blocksensor.getDistance(DistanceUnit.CM) > 5  ) {
            robot.allpower(0.4);
            robot.superintake.setPower(1);
            telemetry.addData("distance from block",robot.blocksensor.getDistance(DistanceUnit.CM) );
            telemetry.update();
        }

        robot.allpower(0);
        robot.superintake.setPower(0.1);

        robot.leftFront.setTargetPosition(LeftFrontEncoder);
        robot.leftBack.setTargetPosition(LeftBackEncoder);
        robot.rightBack.setTargetPosition(RightBackEncoder);
        robot.rightFront.setTargetPosition(RightFrontEncoder);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double speed = 1;
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



        headingdrive.gyroTurn(1, -90);
        liftto.LiftToDistance(1,5, true);
        headingdrive.gyroDrive(1,-10,-90);

        robot.switchableWebcam.setPipeline(robot.redPipeline);

        headingdrive.gyroDrive(1,46, -90);

        headingdrive.gyroTurn(1,-45);

        liftto.LiftToDistance(1,-5, true);


        while (opModeIsActive()) {
            if (robot.redPipeline.getXPosition() > 340) {
                robot.leftBack.setPower(motorspeed);
                robot.leftFront.setPower(motorspeed);
                robot.rightBack.setPower(-motorspeed);
                robot.rightFront.setPower(-motorspeed);
            } else if (robot.redPipeline.getXPosition() < 300) {
                robot.leftBack.setPower(-motorspeed);
                robot.leftFront.setPower(-motorspeed);
                robot.rightBack.setPower(motorspeed);
                robot.rightFront.setPower(motorspeed);
            } else {
                break;
            }



            telemetry.addData("Is Found", robot.redPipeline.isFound());

            if (robot.goldPipeline.isFound()) {
                telemetry.addData("Aligned", robot.redPipeline.getAligned());
                telemetry.addData("Get X", robot.redPipeline.getXPosition());
                telemetry.addData("width", robot.redPipeline.getFoundRect().width);
            }
            telemetry.update();
        }




    }
}

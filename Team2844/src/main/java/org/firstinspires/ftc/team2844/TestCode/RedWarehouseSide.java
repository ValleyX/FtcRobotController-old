package org.firstinspires.ftc.team2844.TestCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

import java.util.PrimitiveIterator;

@Autonomous(name="RedWarehouseSide")
public class RedWarehouseSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 145, 120, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);
        double dist;

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

        headingdrive.gyroDrive(0.7, 30, 0);

        headingdrive.gyroDrive(0.4,-22,0);

        robot.StraifLeft(0.2);
        sleep(200);

        headingdrive.gyroTurn(0.7,-18); //-15

        headingdrive.gyroDrive(0.3,17.5,-18);

        if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Left){
            dist = 5;
            liftto.LiftToDistance(0.9, dist);
        }

        else if (path == RobotHardware.SkystoneDeterminationPipeline.MarkerPosition.Middle) {
            dist = 11;
            liftto.LiftToDistance(0.9, dist);
        }

        else {
            dist = 17;
            liftto.LiftToDistance(0.9, dist);
        }

        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);

        headingdrive.gyroDrive(0.5,-20, -20);

        liftto.LiftToDistance(0.3, -dist + 6);


        headingdrive.gyroTurn(0.75,90);

        headingdrive.gyroDrive(1,60,90);

        robot.StraifRight(0.4);
        sleep(500);

        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);
        robot.rightFront.setPower(0.8);
        robot.rightBack.setPower(0.8);

        sleep(1500);





        /*
        sleep(1000);
        headingdrive.gyroDrive(1,4,-40);
        sleep(510);






        headingdrive.gyroTurn(0.75,90);
        sleep(1000);
        headingdrive.gyroDrive(1,5,90);
        sleep(1000);
        headingdrive.gyroDrive(1,30,90);        //Going into Warehouse and correcting angle
        sleep(1000);
        headingdrive.gyroTurn(1,0);
        sleep(1000);
        headingdrive.gyroTurn(1,0);
        sleep(500);

         */


    }



}

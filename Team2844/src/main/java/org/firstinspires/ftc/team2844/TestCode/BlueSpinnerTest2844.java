package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@Autonomous(name="BlueSpinnerSide")
public class BlueSpinnerTest2844 extends LinearOpMode {
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

        headingdrive.gyroDrive(0.5,30,0);
        //sleep(1000);
        headingdrive.gyroDrive(0.3,-10,0);


        headingdrive.gyroTurn(0.5,-55);

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



        //sleep(500);
        headingdrive.gyroDrive(0.2,13.6,-55);





        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);
        sleep(500);
        headingdrive.gyroDrive(0.5,-9, -30);
        sleep(500);
        liftto.LiftToDistance(0.3, -dist);
        sleep(500);


        headingdrive.gyroTurn(0.3,-80);

        headingdrive.gyroDrive(0.5,-55,-80);

        robot.StraifLeft(0.2);
        sleep(1000);
        //headingdrive.gyroDrive(0.4,-10,-80);

        robot.duckySpins(-0.4);
        //headingdrive.gyroDrive(0.2,-4, 0);
        robot.StraifLeft(0.03);
        sleep(8500);

        robot.StraifRight(0.2);
        sleep(500);

        headingdrive.gyroTurn(0.6, 0);

        headingdrive.gyroDrive(0.2,5,0);


        robot.StraifRight(0.4);
        sleep(1000);

        headingdrive.gyroDrive(1,17 ,0);






    }




}

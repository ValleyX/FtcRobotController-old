package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.LiftDriverTest;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@Autonomous(name="RedSpinnerSide")
public class RedspinnerTest2844 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 320, 240, RobotHardware.cameraSelection.DOWN);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);

        double dist;

        double armdown = 0.87;
        double armup= 0.2;
        double grabin = 0.9;
        double grabout= 0;

        RobotHardware.SkystoneDeterminationPipeline.MarkerPosition path = robot.pipeline.position;

        while (!isStarted())
        {
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


/** step by step description
 *  1. driving up to the alliance hub
 *  2. lifting the lift based on what the camera sensed
 *  3. the intake spits out the cube at the right level
 *     it then backs up a little in order to put the lift down
 *  4. re-adjusts to the right angle for the ducky spinner
 *  5. backs up to the ducky spinner
 *  6. straifs into the wall in order to line up with the ducky spinner and drives back
 *  7. starts spinning the duck while backing up to keep the wheel on the spinner
 *  8. straifs out of the wall
 *  9. goes forward into the storage units.
 */

// 1.

        robot.arm.setPosition(armdown);

        headingdrive.gyroDrive(1,8,0);

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

        /*
        headingdrive.gyroDrive(0.5,30,0);
        //sleep(1000);
        headingdrive.gyroDrive(0.3,-20,0);

         */


        headingdrive.gyroTurn(0.5,35);



        //headingdrive.gyroDrive(0.2,14.1,35);

// 2.

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

        headingdrive.gyroDrive(0.7,12, 35);


        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


// 3.

        robot.superintake.setPower(-1);
        sleep(500);
        robot.superintake.setPower(0);


        headingdrive.gyroDrive(0.5,-9, 30);
        liftto.LiftToDistance(0.3, -dist, false);

// 4.
        headingdrive.gyroTurn(0.8,-90);

        while (robot.liftmotor.isBusy()) {
            idle();
        }
        robot.liftmotor.setPower(0);
        robot.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// 5.


        headingdrive.gyroDrive(0.6,28,-90); //-30.5
        headingdrive.gyroTurn(0.5,0);

// 6.
        robot.StraifLeft(0.5); //straif into the wall
        sleep(1000);
        headingdrive.gyroDrive(0.4,-15, 0);

// 7.
        robot.duckySpins(0.4);
        headingdrive.gyroDrive(0.01, -4, 0 );
        sleep(3000);

// 8.
        robot.StraifLeft(0.4);
        sleep(500);

// 9.

        headingdrive.gyroDrive(0.4, 19 ,0);
        sleep(1000);


    }




}

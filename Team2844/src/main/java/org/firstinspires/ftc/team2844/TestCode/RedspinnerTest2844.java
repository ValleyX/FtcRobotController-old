package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Autonomous(name="RedSpinnerSide")
public class RedspinnerTest2844 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);


        waitForStart();

        headingdrive.gyroDrive(1,10,0);
        //sleep(1000);
        headingdrive.gyroTurn(0.5,30);
        //sleep(500);
        headingdrive.gyroDrive(1,15,30);
        //sleep(500);
        headingdrive.gyroTurn(0.2,50);
        //sleep(500);
        headingdrive.gyroDrive(1,-29.5,50);
        //sleep(500);

        robot.StraifLeft(0.4); //straif into the wall
        sleep(2000);
        headingdrive.gyroDrive(0.4,-10, 0);

        robot.duckySpins(1);
        robot.allpower(-0.01);
        sleep(2500);

        robot.StraifRight(1);
        sleep(100);
        headingdrive.gyroDrive(0.4,16 ,0);




        /*
        headingdrive.gyroTurn(0.5,-15);
        headingdrive.gyroDrive(1,15,-15);


         */



    }




}

package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Autonomous(name="RedSpinnerSide")
public class RedSideWabbleRedTest2844 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);


        waitForStart();

        headingdrive.gyroDrive(1,10,0);
        sleep(1000);
        headingdrive.gyroTurn(0.5,30);
        sleep(500);
        headingdrive.gyroDrive(1,13,30);
        sleep(500);
        headingdrive.gyroTurn(0.2,50);
        sleep(500);
        headingdrive.gyroDrive(1,-27.5,50);
        sleep(500);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.StraifLeft(1);
        sleep(2000);
        headingdrive.gyroDrive(0.5,3,0);

        /*
        headingdrive.gyroTurn(0.5,-15);
        headingdrive.gyroDrive(1,15,-15);


         */



    }




}

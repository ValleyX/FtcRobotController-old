package org.firstinspires.ftc.team2844.TestCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

@Autonomous(name="BlueWarehouseSide")
public class BlueWarehouseSide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);


        waitForStart();

        headingdrive.gyroTurn(0.75,20);
        sleep(500);
        headingdrive.gyroDrive(1,12,20);
        sleep(500);
        headingdrive.gyroTurn(0.75,60);
        sleep(1000);
        headingdrive.gyroDrive(1,1,43);
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

        //Robot dancing
        /*
        headingdrive.gyroTurn(0.6,90);
        sleep(1250);
        headingdrive.gyroTurn(0.85,2);
        sleep(1250);
        headingdrive.gyroDrive(1,1,0);
        sleep(1250);
        headingdrive.gyroTurn(0.5,-90);
        sleep(1250);
        headingdrive.gyroDrive(1,-1,0);
        sleep(1750);
        headingdrive.gyroTurn(0.75,89);
        sleep(1250);
        headingdrive.gyroTurn(0.9,3);
        sleep(1250);
        */



    }

}
package org.firstinspires.ftc.team2844.TestCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Autonomous(name="RedWarehouseSide")
public class RedWarehouseSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest driveto = new DistanceDriverTest(robot, headingdrive);

        waitForStart();
        //Moving to Wabble
        headingdrive.gyroTurn(0.75,-20);
        sleep(500);
        headingdrive.gyroDrive(1,12,-20);
        sleep(500);
        headingdrive.gyroTurn(0.75,-60);
        sleep(1000);
        headingdrive.gyroDrive(1,1,-43);
        sleep(1000);
        headingdrive.gyroDrive(1,-4,-40);
        sleep(510);
        headingdrive.gyroTurn(0.75,90);
        sleep(1000);
        headingdrive.gyroDrive(1,5,90);
        sleep(1000);
        headingdrive.gyroDrive(1,30,90);        //Going into Warehouse and correcting angle
        sleep(1000);
        headingdrive.gyroTurn(1,0);
        sleep(1000);
        sleep(500);
        //headingdrive.gyroTurn(0.5,-125);
       // sleep(500);
       // headingdrive.gyroDrive(1,3,-125);
       // sleep(1000);
       // driveto.DriveToDistance(0.2,1,-125);

    }



}

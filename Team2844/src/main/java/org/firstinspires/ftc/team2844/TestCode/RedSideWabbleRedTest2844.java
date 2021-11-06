package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Autonomous(name="RedSideWabbleRedTest2844")
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
        headingdrive.gyroTurn(0.5,25);
        sleep(500);
        headingdrive.gyroDrive(1,3,25);
        sleep(1000);
        driveto.DriveToDistance(0.2,1.8,25);

    }




}

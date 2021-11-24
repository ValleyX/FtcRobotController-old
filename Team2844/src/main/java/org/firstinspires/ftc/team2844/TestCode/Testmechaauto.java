package org.firstinspires.ftc.team2844.TestCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
@Disabled

@Autonomous(name="Testmechaauto")
public class Testmechaauto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, this,0,0, RobotHardware.cameraSelection.LEFT);

        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest Driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);

        System.out.println("valleyx: im here4");

        waitForStart();
        //encodermecha.StartAction(0.5,5,5,5,true);
        //robot.StraifLeft(1);
        //robot.allpower(1);
       // headingdrive.gyroDrive(0.2,10,0);
       // headingdrive.gyroTurn(0.8,-90);
        //sleep(1000);
        //headingdrive.gyroTurn(0.8,0);
/*
        telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();

        System.out.println("Valley: distance sensor range "+ robot.sensorRange.getDistance(DistanceUnit.INCH));


        encodermecha.StartAction(0.1,10,10,10,true);
        Driveto.DriveToDistance(0.1,1,0);


 */

        liftto.LiftToDistance(0.9,12);
        sleep(5000);
        //liftto.LiftToDistance(0.2,-4);









    }
}

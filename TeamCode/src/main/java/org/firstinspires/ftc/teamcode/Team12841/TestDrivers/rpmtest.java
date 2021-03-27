package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware4motors;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="rpmTest")
public class rpmtest extends LinearOpMode {
    RobotHardware4motors robot;
    public void runOpMode() {
        waitForStart();
        robot = new RobotHardware4motors(hardwareMap, this);

        robot.bucket.setPosition(robot.bucketUp_POS);

        robot.shooterfront.setPower(0.55);
        robot.shooterback.setPower(0.55);
        for (int i = 0; i < 3; i++) {

        rpmThing(1235);
        //sleep(5000);
        }
    }

        public void rpmThing(double targetrpm)
        {
            double checkone;
            double checktwo;
            double frontaveragerpm;
            double checkoneback;
            double checktwoback;
            double backaveragerpm;
            int time = 500; //ms

            do {

                checkone = robot.shooterfront.getCurrentPosition();

                sleep(time);

                checktwo = robot.shooterfront.getCurrentPosition();

                frontaveragerpm = (checktwo - checkone) / ((double)time / 1000.0);
                telemetry.addData("rpmfront: ", frontaveragerpm);


                checkoneback = robot.shooterback.getCurrentPosition();

                sleep(time);

                checktwoback = robot.shooterback.getCurrentPosition();

                backaveragerpm = (checktwoback - checkoneback) / ((double)time / 1000.0);

                telemetry.addData("rpmback: ", backaveragerpm);

                telemetry.update();
            }

            while (frontaveragerpm < targetrpm && backaveragerpm < targetrpm);

                robot.ringpusher.setPosition(robot.backringpusher_POS);
                sleep(1000);
                robot.ringpusher.setPosition(robot.frontringpusher_POS);
                sleep(2000);
        }

    }


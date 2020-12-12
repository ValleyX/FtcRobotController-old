package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

        @Autonomous(name="AutonomousTest")
        public class AutonomousTest1 extends LinearOpMode {
            ColorSensor sensorColor;
            DistanceSensor sensorDistance;
            //EncoderDrive encoder = new EncoderDrive(robot);
            RobotHardware robot;

            float hsvValuesWhite[] = {0F, 0F, 0F};
            float hsvValues[] = {0F, 0F, 0F};

            final float values[] = hsvValues;

            final double SCALE_FACTOR = 255;

            @Override
            public void runOpMode() throws InterruptedException {

                RobotHardware robot = new RobotHardware(hardwareMap, this);
                EncoderDrive encoder = new EncoderDrive(robot);

                waitForStart();

        final double fullturn = 3.14 * 18; //18 inches
        final double halfturn = 3.14 * 9; // 9 inches
        final double quarterturn = 3.14 * 4.5; //4.5 inches

//kinda blue box B (right start)
        //encoder.StartAction(1,97,97,30,true);
        //encoder.StartAction(1, -quarterturn, quarterturn, 5, true);
        //encoder.StartAction(1,3, 3,5, true);

                Color.RGBToHSV(0,0,0, hsvValuesWhite);

                // get a reference to the color sensor.
                sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

                // get a reference to the distance sensor that shares the same name.
                sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

                telemetry.addData("hue",(hsvValuesWhite[0]));
                telemetry.addData("saturation",(hsvValuesWhite[1]));
                telemetry.addData("brightness",(hsvValuesWhite[2]));
                telemetry.update();

                waitForStart();
                robot.rightDrive.setPower(0.15);
                robot.leftDrive.setPower(0.15);

                while (opModeIsActive()) {
                    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                            (int) (sensorColor.green() * SCALE_FACTOR),
                            (int) (sensorColor.blue() * SCALE_FACTOR),
                            hsvValues);

                    telemetry.addData("Alpha", sensorColor.alpha());
                    telemetry.addData("Red  ", sensorColor.red());
                    telemetry.addData("Green", sensorColor.green());
                    telemetry.addData("Blue ", sensorColor.blue());
                    telemetry.addData("Hue", hsvValues[0]);
                    telemetry.update();

                    if (!((hsvValues[0] >=100) && (hsvValues[0] <= 120))) {
                        robot.leftDrive.setPower(0);
                        robot.rightDrive.setPower(0);


                    }
                }

            }
}

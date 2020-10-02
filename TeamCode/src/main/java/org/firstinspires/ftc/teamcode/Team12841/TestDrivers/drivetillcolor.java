package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;



@Autonomous(name="Test: drive till color", group="Test")

public class drivetillcolor extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
     //EncoderDrive encoder = new EncoderDrive(robot);
     RobotHardware robot;

    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;

    @Override
    public void runOpMode() {

        robot = new RobotHardware(hardwareMap, this);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        waitForStart();
        robot.rightDrive.setPower(0.5);
        robot.leftDrive.setPower(0.5);

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

            if ((hsvValues[0] >= 110) && (hsvValues[0] <= 150)) {
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);


            }
        }
    }


}


                // get a reference to the distance sensor that shares the same name.

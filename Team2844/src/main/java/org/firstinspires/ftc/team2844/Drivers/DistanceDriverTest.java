package org.firstinspires.ftc.team2844.Drivers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

public class DistanceDriverTest {
    private RobotHardware robot_;
    private MechaImuDriver imudriver_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    //public DistanceSensor sensorRange;


    /* Constructor setup all class variables here */
    public DistanceDriverTest(RobotHardware robot, MechaImuDriver imuDriver)  {
        robot_ = robot;
        imudriver_ = imuDriver;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        //Rev2mDistanceSensor sensorTimeOfFlight_ = (Rev2mDistanceSensor)sensorRange;
    }

    public void DriveToDistance(double speed,
                                double distancefromobject,
                                double angle) {
/*
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int moveCounts;
        double maxfront;
        double maxback;
        double max;
        double error;
        double steer;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftBackSpeed;
        double rightBackSpeed;
    */

        if (robot_.OpMode_.opModeIsActive()) {
            double currentdistance = robot_.sensorRange.getDistance(DistanceUnit.INCH);
            double distancedrived = -(distancefromobject - currentdistance); // - is temporary
            System.out.println("ValleyXcurrent: "+ currentdistance);
            System.out.println("ValleyXDrived: "+ distancedrived);

            imudriver_.gyroDrive(speed, distancedrived, angle);
            //the problem with the distance sensor might be becasue the angle wasnt correct


        }

    }
}

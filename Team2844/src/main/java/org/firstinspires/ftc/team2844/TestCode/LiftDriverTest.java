package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
/*
public class LiftDriverTest {

    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;


    // Constructor setup all class variables here
    public LiftDriverTest(RobotHardware robot)  {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    public void LiftToDistance (double speed,
                                double distance ) {


        int LiftTarget;
        int moveCounts;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {

            System.out.println("valleyx: im here1");
            robot_.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot_.LIFT_COUNTS_PER_INCH);
            LiftTarget = robot_.leftFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            // used to determine direction of the front

            robot_.liftmotor.setTargetPosition(LiftTarget);

            robot_.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.liftmotor.setPower(speed);

            System.out.println("valleyx: im here2");

            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() &&
                    (robot_.liftmotor.isBusy())) {




            }

            // Stop all motion;
            robot_.liftmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot_.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


}

 */

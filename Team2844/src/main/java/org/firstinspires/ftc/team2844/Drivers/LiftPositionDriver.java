package org.firstinspires.ftc.team2844.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftPositionDriver {

    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;


    // Constructor setup all class variables here
    public LiftPositionDriver(RobotHardware robot)  {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        robot_.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot_.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftToPosition (double speed,
                                double position ) {


        int LiftTarget;
        int moveCounts;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            LiftTarget = (int) (position * robot_.LIFT_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            // used to determine direction of the front

            robot_.liftmotor.setTargetPosition(LiftTarget);

            robot_.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.liftmotor.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() && (robot_.liftmotor.isBusy()) && (robot_.liftdowntouch.getState() == true)) // true means not touched {

                robot_.OpMode_.telemetry.addData("lift position : ", robot_.liftmotor.getCurrentPosition());
                robot_.OpMode_.telemetry.addData("lift target position : ", robot_.liftmotor.getTargetPosition());
                robot_.OpMode_.telemetry.update();

                System.out.println("valleyX: " + robot_.liftmotor.getCurrentPosition());
                System.out.println("valleyX");


            }

            robot_.liftmotor.setPower(0);

            if (robot_.liftdowntouch.getState() == false) {
                robot_.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Turn off RUN_TO_POSITION
            robot_.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }





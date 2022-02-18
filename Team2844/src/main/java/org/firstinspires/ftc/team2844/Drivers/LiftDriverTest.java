package org.firstinspires.ftc.team2844.Drivers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

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
                                double distance,
                                boolean iswaiting ) {


        int LiftTarget;
        int moveCounts;

        waiting_ = iswaiting;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {

            robot_.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot_.LIFT_COUNTS_PER_INCH);
            LiftTarget = robot_.liftmotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            // used to determine direction of the front

            robot_.liftmotor.setTargetPosition(LiftTarget);

            robot_.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.liftmotor.setPower(speed);

            System.out.println("valleyx: im here2");
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            if (waiting_) {
                // keep looping while we are still active, and BOTH motors are running.
                while (robot_.OpMode_.opModeIsActive() &&
                        (robot_.liftmotor.isBusy()) /*&& (robot_.liftdowntouch.getState() == true )*/
                        && (elapsedTime.seconds() < 1)) {

                    robot_.OpMode_.telemetry.addData("lift position : ", robot_.liftmotor.getCurrentPosition());
                    robot_.OpMode_.telemetry.addData("lift target position : ", robot_.liftmotor.getTargetPosition());
                    robot_.OpMode_.telemetry.addData("lift touch : ", robot_.liftdowntouch.getState());

                    robot_.OpMode_.telemetry.update();

                    System.out.println("valleyX: " + robot_.liftmotor.getCurrentPosition());
                    System.out.println("valleyX");

                    if ((robot_.liftdowntouch.getState() == false) && (distance < 0)) {
                        break;
                    }

                }


                // Stop all motion;
                robot_.liftmotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot_.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }


}



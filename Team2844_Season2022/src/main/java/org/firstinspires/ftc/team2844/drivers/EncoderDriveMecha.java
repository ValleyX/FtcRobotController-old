package org.firstinspires.ftc.team2844.drivers;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.RobotHardware;

public class EncoderDriveMecha {

    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public EncoderDriveMecha(RobotHardware robot) {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  If waiting is true then StartAction will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     *  If waiting is false, then the action will start, but it is the caller's
     *  responsibility to loop and check the isActionDone() for completion
     *  and to stop the motors when complete
     *  This feature allow the main program to start up multiple robot actions
     *  in parallel in a larger loop checking multiple robots actions for completion
     */
    public void StartAction(double speed,
                            double leftInches,
                            double rightInches,
                            double timeoutS,
                            boolean waiting) throws InterruptedException //are we returned only when complete?
    {
        waiting_ = waiting;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {
            //setup encoders and motors for this use

            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders");
            robot_.OpMode_.telemetry.update();

            robot_.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d: %7d",
                    robot_.leftFront.getCurrentPosition(),
                    robot_.rightFront.getCurrentPosition(),
                    robot_.leftBack.getCurrentPosition(),
                    robot_.rightBack.getCurrentPosition());
            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot_.leftFront.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightFrontTarget = robot_.rightFront.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            newLeftBackTarget = robot_.leftBack.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightBackTarget = robot_.rightBack.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);

            robot_.leftFront.setTargetPosition(newLeftFrontTarget);
            robot_.rightFront.setTargetPosition(newRightFrontTarget);
            robot_.leftBack.setTargetPosition(newLeftBackTarget);
            robot_.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot_.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.leftFront.setPower(Math.abs(speed));
            robot_.rightFront.setPower(Math.abs(speed));
            robot_.leftBack.setPower(Math.abs(speed));
            robot_.rightBack.setPower(Math.abs(speed));

            runtime_.reset();

            if (waiting_)
            {
                robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot_.leftFront.getCurrentPosition(),
                        robot_.rightFront.getCurrentPosition(),
                        robot_.leftBack.getCurrentPosition(),
                        robot_.rightBack.getCurrentPosition());

                robot_.OpMode_.telemetry.update();
                robot_.OpMode_.idle();
                sleep(5000);

                //then spin here making sure opmode is active, there is available time, action is still running
                while (robot_.OpMode_.opModeIsActive() &&
                        (runtime_.seconds() < timeoutS) &&
                        !IsActionDone())
                {
                    // Display it for the driver.
                    robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                            newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.leftFront.getCurrentPosition(),
                            robot_.rightFront.getCurrentPosition(),
                            robot_.leftBack.getCurrentPosition(),
                            robot_.rightBack.getCurrentPosition());

                    robot_.OpMode_.telemetry.update();
                    robot_.OpMode_.idle();
                }
            }
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return !robot_.leftFront.isBusy() && !robot_.rightFront.isBusy() && !robot_.leftBack.isBusy() && !robot_.rightBack.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion;
        robot_.leftFront.setPower(0);
        robot_.rightFront.setPower(0);
        robot_.leftBack.setPower(0);
        robot_.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }
}



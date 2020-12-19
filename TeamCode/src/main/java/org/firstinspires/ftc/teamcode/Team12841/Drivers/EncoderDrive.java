package org.firstinspires.ftc.teamcode.Team12841.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderDrive
{
    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public EncoderDrive(RobotHardware robot) {
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
                            boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {
            //setup encoders and motors for this use

            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders");
            robot_.OpMode_.telemetry.update();

            robot_.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot_.leftDrive.getCurrentPosition(),
                    robot_.rightDrive.getCurrentPosition());
            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot_.leftDrive.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightTarget = robot_.rightDrive.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            robot_.leftDrive.setTargetPosition(newLeftTarget);
            robot_.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.leftDrive.setPower(Math.abs(speed));
            robot_.rightDrive.setPower(Math.abs(speed));

            runtime_.reset();

            if (waiting_)
            {
                //then spin here making sure opmode is active, there is available time, action is still running
                while (robot_.OpMode_.opModeIsActive() &&
                      (runtime_.seconds() < timeoutS) &&
                      !IsActionDone())
                {
                    // Display it for the driver.
                    robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d",
                            newLeftTarget, newRightTarget);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d",
                            robot_.leftDrive.getCurrentPosition(),
                            robot_.rightDrive.getCurrentPosition());
                    robot_.OpMode_.telemetry.update();
                    robot_.OpMode_.idle();
                }
            }
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return !robot_.leftDrive.isBusy() && !robot_.rightDrive.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion;
        robot_.leftDrive.setPower(0);
        robot_.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot_.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

}

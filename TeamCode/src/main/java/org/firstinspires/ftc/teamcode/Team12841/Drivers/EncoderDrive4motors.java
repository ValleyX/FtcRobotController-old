package org.firstinspires.ftc.teamcode.Team12841.Drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderDrive4motors
{
    private RobotHardware4motors robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public EncoderDrive4motors(RobotHardware4motors robot) {
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

            robot_.leftDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.leftDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.leftDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.leftDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot_.leftDrivefront.getCurrentPosition(),
                    robot_.leftDriveback.getCurrentPosition(),
                    robot_.rightDrivefront.getCurrentPosition(),
                    robot_.rightDrivefront.getCurrentPosition());
            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot_.leftDrivefront.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
           // newLeftTarget = robot_.leftDriveback.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightTarget = robot_.rightDrivefront.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            robot_.leftDrivefront.setTargetPosition(newLeftTarget);
            robot_.leftDriveback.setTargetPosition(newLeftTarget);
            robot_.rightDrivefront.setTargetPosition(newRightTarget);
            robot_.rightDriveback.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot_.leftDrivefront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.leftDriveback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightDrivefront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightDriveback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.leftDrivefront.setPower(Math.abs(speed));
            robot_.leftDriveback.setPower(Math.abs(speed));
            robot_.rightDrivefront.setPower(Math.abs(speed));
            robot_.rightDriveback.setPower(Math.abs(speed));

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
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.leftDrivefront.getCurrentPosition(),
                            robot_.leftDriveback.getCurrentPosition(),
                            robot_.rightDrivefront.getCurrentPosition(),
                            robot_.rightDriveback.getCurrentPosition());
                    robot_.OpMode_.telemetry.update();
                    robot_.OpMode_.idle();
                }
            }
            robot_.leftDrivefront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot_.leftDriveback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot_.rightDrivefront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot_.rightDriveback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
        return !robot_.leftDrivefront.isBusy() && robot_.leftDriveback.isBusy() && !robot_.rightDrivefront.isBusy() && !robot_.rightDriveback.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion;
        robot_.leftDrivefront.setPower(0);
        robot_.leftDriveback.setPower(0);
        robot_.rightDrivefront.setPower(0);
        robot_.rightDriveback.setPower(0);

        // Turn off RUN_TO_POSITION
        robot_.leftDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

}

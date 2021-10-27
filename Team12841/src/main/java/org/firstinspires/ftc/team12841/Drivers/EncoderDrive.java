package org.firstinspires.ftc.team12841.Drivers;

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

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {
            //setup encoders and motors for this use

            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders");
            robot_.OpMode_.telemetry.update();

            robot_.LfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.LbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.RfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.RbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.LfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.LbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.RfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.RbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d ",
                    robot_.LfMotor.getCurrentPosition(),
                    robot_.LbMotor.getCurrentPosition(),
                    robot_.RfMotor.getCurrentPosition(),
                    robot_.RbMotor.getCurrentPosition());
            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot_.LfMotor.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newLeftBackTarget = robot_.LbMotor.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightFrontTarget = robot_.RfMotor.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            newRightBackTarget = robot_.RbMotor.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            robot_.LfMotor.setTargetPosition(newLeftFrontTarget);
            robot_.LbMotor.setTargetPosition(newLeftBackTarget);
            robot_.RfMotor.setTargetPosition(newRightFrontTarget);
            robot_.RbMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot_.LfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.LbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.RfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.RbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.LfMotor.setPower(Math.abs(speed));
            robot_.LbMotor.setPower(Math.abs(speed));
            robot_.RfMotor.setPower(Math.abs(speed));
            robot_.RbMotor.setPower(Math.abs(speed));

            runtime_.reset();

            if (waiting_)
            {
                //then spin here making sure opmode is active, there is available time, action is still running
                while (robot_.OpMode_.opModeIsActive() &&
                      (runtime_.seconds() < timeoutS) &&
                      !IsActionDone())
                {
                    // Display it for the driver.
                    robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                            newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget ,newRightBackTarget);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.LfMotor.getCurrentPosition(),
                            robot_.LbMotor.getCurrentPosition(),
                            robot_.RfMotor.getCurrentPosition(),
                            robot_.RbMotor.getCurrentPosition());
                    robot_.OpMode_.telemetry.update();
                    robot_.OpMode_.idle();
                }
                StopAction();
            }
        }
    }

    //check if the motors have hit their target
    public boolean IsActionDone()
    {
      //  return !robot_.LfMotor.isBusy() && !robot_.LbMotor.isBusy() && !robot_.RfMotor.isBusy() && !robot_.RbMotor.isBusy();
        return !robot_.LfMotor.isBusy() || !robot_.LbMotor.isBusy() || !robot_.RfMotor.isBusy() || !robot_.RbMotor.isBusy();
    }
//Hello people
    //stop the motors
    public void StopAction()
    {
        // Stop all motion;
        robot_.LfMotor.setPower(0);
        robot_.LbMotor.setPower(0);
        robot_.RfMotor.setPower(0);
        robot_.RbMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot_.LfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.LbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.RfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.RbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }

}

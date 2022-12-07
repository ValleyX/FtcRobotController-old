package org.firstinspires.ftc.team22076;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    This Class will move the robot the given number of inches
 */
public class EncoderDrive {

    private RobotHardware robot_;  //This is the LinearOpMode
    private ElapsedTime runtime_;  //This keeps track of the amount of time a function will run
    private boolean waiting_;  //This will tell functions to wait to complete or return immediately


    //Constructor
    public EncoderDrive(RobotHardware Robot) {
        robot_ = Robot;
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
    public void MoveInches(double speed,
                            double leftInches,
                            double rightInches,
                            double timeoutS,
                            boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        //Variables for Encoder counts targets
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {
            //setup encoders and motors for this use

            //write telemetry to Driver Console
            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders");
            robot_.OpMode_.telemetry.update();

            //Reset Motor Encoders
            robot_.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Tell motor control to use Encoder to track where you are
            robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            robot_.OpMode_.telemetry.addData("Path0:", "Starting at %7d :%7d :%7d :%7d ",
                    robot_.leftFront.getCurrentPosition(),
                    robot_.leftBack.getCurrentPosition(),
                    robot_.rightFront.getCurrentPosition(),
                    robot_.rightBack.getCurrentPosition());
            robot_.OpMode_.telemetry.update();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot_.leftFront.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newLeftBackTarget = robot_.leftBack.getCurrentPosition() + (int) (leftInches * robot_.COUNTS_PER_INCH);
            newRightFrontTarget = robot_.rightFront.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);
            newRightBackTarget = robot_.rightBack.getCurrentPosition() + (int) (rightInches * robot_.COUNTS_PER_INCH);

            //Tell motor controller what count we want to move to
            robot_.leftFront.setTargetPosition(newLeftFrontTarget);
            robot_.leftBack.setTargetPosition(newLeftBackTarget);
            robot_.rightFront.setTargetPosition(newRightFrontTarget);
            robot_.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION mode in motor controller
            robot_.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot_.leftFront.setPower(Math.abs(speed));
            robot_.leftBack.setPower(Math.abs(speed));
            robot_.rightFront.setPower(Math.abs(speed));
            robot_.rightBack.setPower(Math.abs(speed));

            runtime_.reset(); //reset timer

            if (waiting_)  //Check if waiting for position to be reached or set and forget
            {
                //then spin here making sure opmode is active, there is available time, action is still running
                while (robot_.OpMode_.opModeIsActive() &&
                        (runtime_.seconds() < timeoutS) &&
                        !IsActionDone()) {
                    // Display it for the driver.
                    robot_.OpMode_.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                            newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                    robot_.OpMode_.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot_.leftFront.getCurrentPosition(),
                            robot_.leftBack.getCurrentPosition(),
                            robot_.rightFront.getCurrentPosition(),
                            robot_.rightBack.getCurrentPosition());
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
        return !robot_.leftFront.isBusy() || !robot_.leftBack.isBusy() || !robot_.rightFront.isBusy() || !robot_.rightBack.isBusy();
    }

    //stop the motors
    public void StopAction()
    {
        // Stop all motion;
        robot_.leftFront.setPower(0);
        robot_.leftBack.setPower(0);
        robot_.rightFront.setPower(0);
        robot_.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION, this allows set power to move motors in telop
        robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot_.OpMode_.idle();   //give the processor time to act
        waiting_ = false;
    }


    /*
    The function will take speed and angle in degrees and turn the robot.
    The angle is 0 degrees at start, Negative degrees will turn counterclockwise.
    Positive degrees will turn clockwise.
    */
    public void MoveDirection(double speed, int degrees) {

        double LeftInches;
        double RightInches;

        if (degrees > 0) {  //need to move clockwise so left wheels forward, right wheels backward
          LeftInches = robot_.INCHES_PER_DEGREE * degrees;
          RightInches = -robot_.INCHES_PER_DEGREE * degrees;
        }
        else if(degrees < 0 ) {//need to move counterclockwise so left wheels backward, right wheels forward
          LeftInches = -robot_.INCHES_PER_DEGREE * degrees;
          RightInches = robot_.INCHES_PER_DEGREE * degrees;
        }
        else {
            LeftInches = 0;
            RightInches = 0;
        }

        MoveInches(speed, LeftInches, RightInches, 5, true);
    }
}
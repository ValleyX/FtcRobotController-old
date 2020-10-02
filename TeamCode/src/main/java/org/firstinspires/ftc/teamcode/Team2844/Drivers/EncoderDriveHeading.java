package org.firstinspires.ftc.teamcode.Team2844.Drivers;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class EncoderDriveHeading
{

    //static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    //static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    //static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double P_DRIVE_COEFF = 0.095;     // Larger is more responsive, but also less stable

    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public EncoderDriveHeading(RobotHardware robot) {
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
                            double distance,
                            double heading,
                            double timeoutS,
                            boolean waiting) //are we returned only when complete?
    {
        waiting_ = waiting;

        int newLeftTarget;
        //int newLeftBackTarget;
        int newRightTarget;
        //int newRightBackTarget;

        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        if (heading == 0)
            heading = 0.1;

        heading = -heading;

        // Ensure that the opmode is still active

        if (robot_.OpMode_.opModeIsActive())
        {
            //setup encoders and motors for this use
            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders ");
            robot_.OpMode_.telemetry.update();

            robot_.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot_.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Send telemetry message to indicate successful Encoder reset

            robot_.OpMode_.telemetry.addData("Path0", "Starting at :%7d :%7d",

                    robot_.leftDrive.getCurrentPosition(),

                    //robot_.leftBackDrive.getCurrentPosition(),

                    robot_.rightDrive.getCurrentPosition());


            robot_.OpMode_.telemetry.update();


            // Determine new target position, and pass to motor controller

            newLeftTarget = robot_.leftDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            //newLeftBackTarget = robot_.leftBackDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            newRightTarget = robot_.rightDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            ///newRightBackTarget = robot_.rightBackDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            robot_.leftDrive.setTargetPosition(newLeftTarget);

            //robot_.leftBackDrive.setTargetPosition(newLeftBackTarget);

            robot_.rightDrive.setTargetPosition(newRightTarget);

            //robot_.rightBackDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION

            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.

            robot_.leftDrive.setPower(Math.abs(speed));

            //robot_.leftBackDrive.setPower(Math.abs(speed));

            robot_.rightDrive.setPower(Math.abs(speed));

            //robot_.rightBackDrive.setPower(Math.abs(speed));


            runtime_.reset();


            if (waiting_) {

                //then spin here making sure opmode is active, there is available time, action is still running

                while (robot_.OpMode_.opModeIsActive() &&

                        (runtime_.seconds() < timeoutS) &&

                        !IsActionDone()) {

                    // adjust relative speed based on heading error.

                    error = getError(heading);

                    steer = getSteer(error, P_DRIVE_COEFF);


                    // if driving in reverse, the motor correction also needs to be reversed

                    if (distance < 0)

                        steer *= -1.0;


                    leftSpeed = speed - steer;

                    rightSpeed = speed + steer;


                    // Normalize speeds if either one exceeds +/- 1.0;

                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                    if (max > 1.0) {

                        leftSpeed /= max;

                        rightSpeed /= max;

                    }


                    robot_.leftDrive.setPower(Math.abs(leftSpeed));

                    //robot_.leftBackDrive.setPower(Math.abs(leftSpeed));

                    robot_.rightDrive.setPower(Math.abs(rightSpeed));

                    //robot_.rightBackDrive.setPower(Math.abs(rightSpeed));


                    // Display it for the driver.

                    robot_.OpMode_.telemetry.addData("Path1", "Running to :%7d :%7d",

                            newLeftTarget, newRightTarget);

                    robot_.OpMode_.telemetry.addData("Path2", "Running at :%7d :%7d",

                            robot_.leftDrive.getCurrentPosition(),

                            //robot_.leftBackDrive.getCurrentPosition(),

                            robot_.rightDrive.getCurrentPosition());

                    robot_.OpMode_.telemetry.update();

                    robot_.OpMode_.idle();

                }

                StopAction();

            }

        }

    }


    public void StartActionCoef(double speed,

                                double distance,

                                double heading,

                                double timeoutS,

                                boolean waiting, //are we returned only when complete?

                                double coef) {

        waiting_ = waiting;


        int newLeftTarget;

        //int newLeftBackTarget;

        int newRightTarget;

        //int newRightBackTarget;


        double max;

        double error;

        double steer;

        double leftSpeed;

        double rightSpeed;


        if (heading == 0)

            heading = 0.1;


        heading = -heading;


        // Ensure that the opmode is still active

        if (robot_.OpMode_.opModeIsActive()) {

            //setup encoders and motors for this use


            robot_.OpMode_.telemetry.addData("Status", "Resetting Encoders ");

            robot_.OpMode_.telemetry.update();


            robot_.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //robot_.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot_.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Send telemetry message to indicate successful Encoder reset

            robot_.OpMode_.telemetry.addData("Path0", "Starting at :%7d :%7d",

                    robot_.leftDrive.getCurrentPosition(),

                    //robot_.leftBackDrive.getCurrentPosition(),

                    robot_.rightDrive.getCurrentPosition());


            robot_.OpMode_.telemetry.update();


            // Determine new target position, and pass to motor controller

            newLeftTarget = robot_.leftDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            //newLeftBackTarget = robot_.leftBackDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            newRightTarget = robot_.rightDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            //newRightBackTarget = robot_.rightBackDrive.getCurrentPosition() + (int) (distance * robot_.COUNTS_PER_INCH);

            robot_.leftDrive.setTargetPosition(newLeftTarget);

            //robot_.leftBackDrive.setTargetPosition(newLeftBackTarget);

            robot_.rightDrive.setTargetPosition(newRightTarget);

            //robot_.rightBackDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION

            robot_.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.

            robot_.leftDrive.setPower(Math.abs(speed));

            //robot_.leftBackDrive.setPower(Math.abs(speed));

            robot_.rightDrive.setPower(Math.abs(speed));

            //robot_.rightBackDrive.setPower(Math.abs(speed));


            runtime_.reset();


            if (waiting_) {

                //then spin here making sure opmode is active, there is available time, action is still running

                while (robot_.OpMode_.opModeIsActive() &&

                        (runtime_.seconds() < timeoutS) &&

                        !IsActionDone()) {

                    // adjust relative speed based on heading error.

                    error = getError(heading);


                    if (Math.abs(error) < 4) {

                        coef = P_DRIVE_COEFF;

                    }


                    steer = getSteer(error, coef);


                    // if driving in reverse, the motor correction also needs to be reversed

                    if (distance < 0)

                        steer *= -1.0;


                    leftSpeed = speed - steer;

                    rightSpeed = speed + steer;


                    // Normalize speeds if either one exceeds +/- 1.0;

                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                    if (max > 1.0) {

                        leftSpeed /= max;

                        rightSpeed /= max;

                    }


                    robot_.leftDrive.setPower(Math.abs(leftSpeed));

                    //robot_.leftBackDrive.setPower(Math.abs(leftSpeed));

                    robot_.rightDrive.setPower(Math.abs(rightSpeed));

                    //robot_.rightBackDrive.setPower(Math.abs(rightSpeed));


                    // Display it for the driver.

                    robot_.OpMode_.telemetry.addData("Path1", "Running to :%7d :%7d",

                            newLeftTarget, newRightTarget);

                    robot_.OpMode_.telemetry.addData("Path2", "Running at :%7d :%7d",

                            robot_.leftDrive.getCurrentPosition(),

                            robot_.rightDrive.getCurrentPosition());

                    robot_.OpMode_.telemetry.update();

                    robot_.OpMode_.idle();

                }

                StopAction();

            }

        }

    }


    //check if the motors have hit their target

    public boolean IsActionDone() {

        //return !robot_.leftFrontDrive.isBusy() && !robot_.leftBackDrive.isBusy() && !robot_.rightFrontDrive.isBusy() && !robot_.rightBackDrive.isBusy();

        return !robot_.leftDrive.isBusy() || !robot_.rightDrive.isBusy();

        //return !robot_.leftFrontDrive.isBusy();

    }


    //stop the motors

    public void StopAction() {

        // Stop all motion;

        robot_.leftDrive.setPower(0);

        //robot_.leftBackDrive.setPower(0);

        robot_.rightDrive.setPower(0);

        //robot_.rightBackDrive.setPower(0);


        // Turn off RUN_TO_POSITION

        System.out.println("ValleyX set RUN_WITHOUT_ENCODER");

        robot_.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot_.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot_.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot_.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot_.OpMode_.idle();   //give the processor time to act

        waiting_ = false;

    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * <p>
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {


        double robotError;


        // calculate error in -179 to +180 range  (

        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        robotError = targetAngle - gyroActual;

        System.out.println("ValleyX variables Robot error= " + robotError + " target angle= " + targetAngle + " gyro actual= " + gyroActual);

        while (robotError > 180) robotError -= 360;

        while (robotError <= -180) robotError += 360;

        return robotError;

    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);

    }
}
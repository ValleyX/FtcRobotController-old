package org.firstinspires.ftc.team2844.Drivers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

public class MechaImuDriver {

    private RobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;

    /* Constructor setup all class variables here */
    public MechaImuDriver(RobotHardware robot) {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;

    }
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

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

        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {

            robot_.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot_.COUNTS_PER_INCH);
            newLeftFrontTarget = robot_.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot_.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot_.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot_.rightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            /** used to determine direction of the front
             * */
            robot_.leftFront.setTargetPosition(newLeftFrontTarget);
            robot_.rightFront.setTargetPosition(newRightFrontTarget);
            robot_.leftBack.setTargetPosition(newLeftBackTarget);
            robot_.rightBack.setTargetPosition(newRightBackTarget);

            robot_.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot_.leftFront.setPower(speed);
            robot_.rightFront.setPower(speed);
            robot_.leftBack.setPower(speed);
            robot_.rightBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() &&
                    (robot_.leftFront.isBusy() && robot_.rightFront.isBusy() && robot_.leftBack.isBusy() && robot_.rightBack.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot_.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftFrontSpeed = speed - steer;
                leftBackSpeed = speed - steer;
                rightFrontSpeed = speed + steer;
                rightBackSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
                //maxback = Math.max(Math.abs(leftBackSpeed);//, Math.abs(rightBackSpeed));
                //max = maxback + maxfront;

                if (max > 1.0/* && maxback > 1.0*/) {
                    leftFrontSpeed /= max;
                    rightBackSpeed /= max;
                    leftBackSpeed /= max;
                    rightFrontSpeed /= max;
                }



                robot_.leftFront.setPower(leftFrontSpeed);
                robot_.rightFront.setPower(rightFrontSpeed);
                robot_.leftBack.setPower(leftBackSpeed);
                robot_.rightBack.setPower(rightBackSpeed);

                // Display drive status for the driver.
                robot_.OpMode_.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                robot_.OpMode_.telemetry.addData("Targetfront", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget);
                robot_.OpMode_.telemetry.addData("Targetback", "%7d:%7d", newLeftBackTarget, newRightBackTarget);

                robot_.OpMode_.telemetry.addData("Actualfront", "%7d:%7d", robot_.leftFront.getCurrentPosition(),robot_.rightFront.getCurrentPosition());
                robot_.OpMode_.telemetry.addData("Actualback", "%7d:%7d", robot_.leftBack.getCurrentPosition(), robot_.rightBack.getCurrentPosition());

                robot_.OpMode_.telemetry.addData("Speedfront", "%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed);
                robot_.OpMode_.telemetry.addData("Speedback", "%5.2f:%5.2f", leftBackSpeed, rightBackSpeed);

                robot_.OpMode_.telemetry.update();


            }

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
        }
        /*
        robot_.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot_.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot_.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot_.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
    }

    /*

/**
*  Method to spin on central axis to point in a new direction.
*  Move will stop if either of these conditions occur:
*  1) Move gets to the heading (angle)
*  2) Driver stops the opmode running.
*
* @param speed Desired speed of turn.
* @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
*                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
*                   If a relative angle is required, add/subtract from current heading.
*/

    public void gyroTurn (  double speed, double angle) {

// keep looping while we are still active, and not on heading.
        while (robot_.OpMode_.opModeIsActive() && !onHeading(speed, angle, robot_.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            robot_.OpMode_.telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     *                   */

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

// keep looping while we have time remaining.
        holdTimer.reset();
        while (robot_.OpMode_.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, robot_.P_TURN_COEFF);
            robot_.OpMode_.telemetry.update();
        }

// Stop all motion;
        robot_.leftFront.setPower(0);
        robot_.rightFront.setPower(0);
        robot_.leftBack.setPower(0);
        robot_.leftBack.setPower(0);
    }

    /*
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftBackSpeed;
        double rightBackSpeed;


// determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot_.HEADING_THRESHOLD) {
            steer= 0;
            leftBackSpeed  = 0.0;
            rightBackSpeed = 0.0;
            leftFrontSpeed  = 0.0;
            rightFrontSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightBackSpeed  = speed * steer;
            rightFrontSpeed  = speed * steer; // no -
            leftBackSpeed   = -rightBackSpeed; // yep -
            leftFrontSpeed = -rightFrontSpeed;
        }

// Send desired speeds to motors.
        robot_.leftFront.setPower(leftFrontSpeed);
        robot_.rightFront.setPower(rightFrontSpeed);
        robot_.leftBack.setPower(leftBackSpeed);
        robot_.rightBack.setPower(rightBackSpeed);

// Display it for the driver.
    /*
telemetry.addData("Target", "%5.2f", angle);
telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
telemetry.addData("Speed.", "%5.2f:%5.2f:%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);


     */

        //robot_.OpMode_.telemetry.update();
        return onTarget;


    }

    /*
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles);


        double gyroActual = -robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        robotError = targetAngle - gyroActual;


        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        //robotError = targetAngle - gyro.getIntegratedZValue();

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
/**
* used for debugging angles
        robot_.OpMode_.telemetry.addData("current angel is", gyroActual);
        robot_.OpMode_.telemetry.addData("robot error is", robotError);
        robot_.OpMode_.telemetry.addData("targetangle is", targetAngle);
        robot_.OpMode_.telemetry.update();

        System.out.println("ValleyX: current angel is" + gyroActual);
        System.out.println("ValleyX: robot error is"+ robotError);
        System.out.println("ValleyX: ctargetangle is"+ targetAngle);


 */
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



}

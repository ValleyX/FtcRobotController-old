/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcore.internal.android.dx.cf.code.Simulator;
//import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;

import java.nio.file.Watchable;
import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Gyro")
public class MechabotAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot_;


    BNO055IMU imu = null;
    /*
        static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /

                                                        (WHEEL_DIAMETER_INCHES * 3.1415);


     */
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        robot_ = new RobotHardware(hardwareMap, this, 0, 0, RobotHardware.cameraSelection.LEFT);
       // imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*

         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        //gryo = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot_.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot_.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot_.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot_.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

 */
/*
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating imu");    //
        telemetry.update();

        imu.initialize(parameters);
*/
        // make sure the gyro is calibrated before continuing

/*
        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }


 */
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
        //    telemetry.addData(">", "Robot Heading = %d", gryo.getIntegratedZValue());
        //    telemetry.update();
        }
*/
        //gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        waitForStart();



        gyroDrive(DRIVE_SPEED, 5.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, 90.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, 90.0, 1);    // Hold -45 Deg heading for a 1/2 second

        /*
        gyroDrive(DRIVE_SPEED, -5.0, 45.0);  // Drive FWD 12 inches at 45 degrees
        gyroTurn( TURN_SPEED,  0.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  0, 1);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,   -45);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   -45, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-5.0, -45);    // Drive REV 48 inches

         */





        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
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
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot_.COUNTS_PER_INCH);
            newLeftFrontTarget = robot_.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot_.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot_.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot_.rightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
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
            while (opModeIsActive() &&
                    (robot_.leftFront.isBusy() && robot_.rightFront.isBusy() && robot_.leftBack.isBusy() && robot_.rightBack.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftFrontSpeed = speed - steer;
                leftBackSpeed = speed - steer;
                rightFrontSpeed = speed + steer;
                rightBackSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                maxfront = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
                maxback = Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed));
                max = maxback + maxfront;

                if (maxfront > 1.0 && maxback > 1.0) {
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
                //telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
               // telemetry.addData("Target", "%7d:%7d:%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                //telemetry.addData("Actual", "%7d:%7d:%7d:%7d", robot_.leftFront.getCurrentPosition(),robot_.rightFront.getCurrentPosition(), robot_.leftBack.getCurrentPosition(), robot_.rightBack.getCurrentPosition());

                //telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
                telemetry.update();


            }

            // Stop all motion;
            robot_.leftFront.setPower(0);
            robot_.rightFront.setPower(0);
            robot_.leftBack.setPower(0);
            robot_.leftBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot_.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot_.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /*

/**
*  Method to spin on central axis to point in a new direction.
*  Move will stop if either of these conditions occur:
*  1) Move gets to the heading (angle)
*  2) Driver stops the opmode running.
*
* @param speed Desired speed of turn.
* @param angle      Absolutye Angle (in Degrees) relative to last gyro reset.
*                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
*                   If a relative angle is required, add/subtract from current heading.
*/

public void gyroTurn (  double speed, double angle) {

// keep looping while we are still active, and not on heading.
while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
   // Update telemetry & Allow time for other processes to run.
   telemetry.update();
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
while (opModeIsActive() && (holdTimer.time() < holdTime)) {
   // Update telemetry & Allow time for other processes to run.
   onHeading(speed, angle, P_TURN_COEFF);
   telemetry.update();
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

if (Math.abs(error) <= HEADING_THRESHOLD) {
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

    telemetry.update();
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


        double gyroActual = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        robotError = targetAngle - gyroActual;


        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        //robotError = targetAngle - gyro.getIntegratedZValue();

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;

        telemetry.addData("current angel is", gyroActual);
        telemetry.addData("robot error is", robotError);
        telemetry.addData("targetangle is", targetAngle);
        telemetry.update();

        System.out.println("ValleyX: current angel is" + gyroActual);
        System.out.println("ValleyX: robot error is"+ robotError);
        System.out.println("ValleyX: ctargetangle is"+ targetAngle);

        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */

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

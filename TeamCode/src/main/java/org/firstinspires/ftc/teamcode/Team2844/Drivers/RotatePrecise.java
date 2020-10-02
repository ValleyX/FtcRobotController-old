package org.firstinspires.ftc.teamcode.Team2844.Drivers;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;



public class RotatePrecise {

    private RobotHardware robot_;

    private ElapsedTime runtime_;

    private boolean waiting_;



    /* Constructor setup all class variables here */

    public RotatePrecise(RobotHardware robot) {

        robot_ = robot;

        runtime_ = new ElapsedTime();

        waiting_ = false;

//hey sam, it hannah

        /* ---new remapping code --*/

        //swapping y & z axis due to vertical mounting of rev expansion board

        //byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes

        byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis


        //Need to be in CONFIG mode to write to registers

        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);


        robot_.OpMode_.sleep(100); //Changing modes requires a delay before doing anything else


        //Write to the AXIS_MAP_CONFIG register

        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE);


        //Write to the AXIS_MAP_SIGN register

        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);


        //Need to change back into the IMU mode to use the gyro

        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);


        robot_.OpMode_.sleep(100); //Changing modes again requires a delay

        /* ---new remapping code ---*/


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // inertial motion unit


        parameters.mode = BNO055IMU.SensorMode.IMU;

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = false;


        robot_.imu.initialize(parameters);


        // make sure the imu gyro is calibrated before continuing.

        while (!robot_.OpMode_.isStopRequested() && !robot_.imu.isGyroCalibrated()) {

            robot_.OpMode_.sleep(50);

            robot.OpMode_.idle();

        }

        if (!robot_.imu.isGyroCalibrated()) {

            System.out.println("ValleyX: Gyro not calibrated");

        }


        System.out.println("ValleyX: imu calib status" + robot_.imu.getCalibrationStatus().toString());

        robot_.OpMode_.telemetry.addData("Mode", "calibrated");

        robot_.OpMode_.telemetry.update();

        float gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


    }


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port

    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",

    // and named "imu".


    public void RotatePrecise(double gyroTarget, double gyroRange, double minSpeed, double addSpeed, int timesCorrect) {

        double turnPower = 0;

        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //System.out.println("ValleyX RotatePrecise initial gyroActual " + gyroActual);

        gyroTarget = -gyroTarget;

        gyroTarget += gyroActual + 360.0;

        //robot_.OpMode_.telemetry.addData("gyroActual", Double.toString(gyroActual));

        //System.out.println("ValleyX gyroActual " + gyroActual);

        gyroTarget %= 360;

        //System.out.println("ValleyX RotatePrecise initial gyroTarget " + gyroTarget);

        int correctCount = 0;


        while ((correctCount < timesCorrect) && robot_.OpMode_.opModeIsActive()) {

            //System.out.println("ValleyX starting while loop in rotatePrecise");

            gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360

            //System.out.println("ValleyX delta value before adjustment " + delta);


            if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180


            //System.out.println("ValleyX delta value after adjustment" + delta + " gyro range " + gyroRange);

            if (Math.abs(delta) > gyroRange) { //checks if delta is out of range

                correctCount = 0;

                double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees

                if (Math.abs(gyroMod) > 1.0)

                    gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees

                turnPower = minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod;

            } else {

                correctCount++;

                turnPower = 0;

            }

            double leftPower, rightPower;

            //System.out.println("ValleyX gyroTarget sam " + gyroTarget);

            if (gyroTarget > 0) {   // turn right

                leftPower = -turnPower;

                rightPower = turnPower;

            } else {   // turn left

                leftPower = turnPower;

                rightPower = -turnPower;

            }


            //System.out.println("ValleyX leftPower " + leftPower + " rightPower " + rightPower);

            // set power to rotate.

            robot_.leftDrive.setPower(leftPower);

            //robot_.leftBackDrive.setPower(leftPower);

            robot_.rightDrive.setPower(rightPower);

            //robot_.rightBackDrive.setPower(rightPower);

            robot_.OpMode_.idle();

        }

        //   return this.correctCount;

    }
}
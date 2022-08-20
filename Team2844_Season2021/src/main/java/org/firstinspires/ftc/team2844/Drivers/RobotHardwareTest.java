package org.firstinspires.ftc.team2844.Drivers;

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus.GoldAlignDetectorTry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 *
 */
public class RobotHardwareTest
{
//    public static final double P_DRIVE_COEFF = 2;
    public LinearOpMode OpMode_;

    public DcMotor  leftFront;
    public DcMotor  rightFront;
    public DcMotor  leftBack;
    public DcMotor duckySpinner;
    public DcMotor  rightBack;
    public DcMotor liftmotor;
    public DcMotor superintake;
    public DistanceSensor sensorRange;



    public WebcamName webcamLeft; //
    public WebcamName webcamRight; //
    public OpenCvSwitchableWebcam switchableWebcam;
    public SkystoneDeterminationPipeline pipeline;
    public GoldAlignDetectorTry goldPipeline;

    public enum cameraSelection
    {
        LEFT,
        RIGHT
    }

    BNO055IMU imu = null;


    //encoder
    public final double     COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     DRIVE_GEAR_REDUCTION    = 20;     // This is < 1.0 if geared UP
    public final double     ONE_MOTOR_COUNT         = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public final double     Distance_in_one_rev     = 4.0  * Math.PI; //in
    public final double     COUNTS_PER_INCH         = ONE_MOTOR_COUNT / Distance_in_one_rev ;  //TODO determine// in class

    //imu (turny thingy)
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.07;     // Larger is more responsive, but also less stable 0.1
    static final double P_DRIVE_COEFF = 0.015;     // Larger is more responsive, but also less stable 0.15

    //lift
    public final double     LIFT_COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    public final double     LIFT_DRIVE_GEAR_REDUCTION    = 30.0;     // This is < 1.0 if geared UP  //original was 20
    public final double     LIFT_ONE_MOTOR_COUNT         = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public final double     LIFT_DISTANCE_IN_ONE_REV     = 2.7* Math.PI; //actual bot is 9.5
    public final double     LIFT_COUNTS_PER_INCH         = LIFT_ONE_MOTOR_COUNT / LIFT_DISTANCE_IN_ONE_REV ;  //TODO determine// in class



    /* Constructor */
    public RobotHardwareTest(HardwareMap ahwMap, LinearOpMode opMode, int x, int y, final cameraSelection camera) {
        /* Public OpMode members. */
        OpMode_ = opMode;

        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());
        webcamLeft = OpMode_.hardwareMap.get(WebcamName.class, "Webcam 1"); // USB 3.0
        //webcamRight = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new RobotHardwareTest.SkystoneDeterminationPipeline(x, y);
        goldPipeline = new GoldAlignDetectorTry();

       // switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamRight);
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamLeft);
       // switchableWebcam.openCameraDevice();
        switchableWebcam.setPipeline(pipeline);
        //switchableWebcam.setPipeline(goldPipeline);
/*
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired camera here
                if (camera == RobotHardwareTest.cameraSelection.LEFT) {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcamLeft);
                } else {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(webcamLeft);
                }
            }
        });
*/
        // Define and Initialize Motors
         leftFront = ahwMap.get(DcMotor.class,"leftFront");
         rightFront = ahwMap.get(DcMotor.class,"rightFront");
         leftBack = ahwMap.get(DcMotor.class,"leftBack");
         rightBack = ahwMap.get(DcMotor.class,"rightBack");
        duckySpinner = ahwMap.get(DcMotor.class, "duckSpinner");
         liftmotor = ahwMap.get(DcMotor.class, "liftMotor");
         superintake = ahwMap.get(DcMotor.class, "superintake");


         //test
        //sensorRange = ahwMap.get(DistanceSensor.class, "distance");
        //test


            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        liftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        // Set all motors to run without encoders by default
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        ///TEST CODE
        while (!OpMode_.isStopRequested() && !imu.isGyroCalibrated()) {
            OpMode_.sleep(50);
            OpMode_.idle();
        }

        if (!imu.isGyroCalibrated()) {
            System.out.println("ValleyX: Gyro not calibrated");
        }

        System.out.println("ValleyX: imu calib status" + imu.getCalibrationStatus().toString());
        OpMode_.telemetry.addData("Mode", "calibrated");
        OpMode_.telemetry.update();
        ///TEST CODE

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        //An enum to define the ring stack size
        public enum MarkerPosition
        {
            Left,
            Middle,
            Right
        }
        //Some color constants
        static final Scalar BLUE = new Scalar(0, 255, 0);
        static final Scalar GREEN = new Scalar(255, 255, 255);
        //Team Color Blue = 109 Average

        //The core values which define the location and size of the sample regions
        //box location and dimensions

        //Point REGION1_TOPLEFT_ANCHOR_POINT;

        Point REGION1_TOPLEFT_ANCHOR_POINTLEFT;
        Point REGION1_TOPLEFT_ANCHOR_POINTMIDDLE;
        Point REGION1_TOPLEFT_ANCHOR_POINTRIGHT;


        static final int REGION_WIDTH = 60;
        static final int REGION_HEIGHT = 60;

        public final int  FOUR_RING_THRESHOLD = 150;

        public final int  ONE_RING_THRESHOLD = 135;

        Point region1Middle_pointA;
        Point region1Middle_pointB;

        Point region1Left_pointA;
        Point region1Left_pointB;

        Point region1Right_pointA;
        Point region1Right_pointB;




        //Working variables
        Mat region1Middle_Cb;
        Mat region1Left_Cb;
        Mat region1Right_Cb;



        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgMiddle;

        int avgLeft;

        int avgRight;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile MarkerPosition position = MarkerPosition.Middle;
        public volatile int SkystoneAverageMiddle;
        public volatile int SkystoneAverageLeft;
        public volatile int SkystoneAverageRight;


        public SkystoneDeterminationPipeline(int x, int y)
        {
            /*
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
             */
            REGION1_TOPLEFT_ANCHOR_POINTMIDDLE = new Point(x, y); // 200, 165
            region1Middle_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.x, REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.y);
            region1Middle_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTMIDDLE.y + REGION_HEIGHT);

            REGION1_TOPLEFT_ANCHOR_POINTLEFT = new Point(2, y); // 200, 165
            region1Left_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTLEFT.x, REGION1_TOPLEFT_ANCHOR_POINTLEFT.y);
            region1Left_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTLEFT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTLEFT.y + REGION_HEIGHT);

            REGION1_TOPLEFT_ANCHOR_POINTRIGHT = new Point(259, y); // 200, 165
            region1Right_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINTRIGHT.x, REGION1_TOPLEFT_ANCHOR_POINTRIGHT.y);
            region1Right_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINTRIGHT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINTRIGHT.y + REGION_HEIGHT);


        }

        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1Middle_Cb = Cb.submat(new Rect(region1Middle_pointA, region1Middle_pointB));
            region1Left_Cb = Cb.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Right_Cb = Cb.submat(new Rect(region1Right_pointA, region1Right_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avgMiddle = (int) Core.mean(region1Middle_Cb).val[0];
            avgLeft = (int) Core.mean(region1Left_Cb).val[0];
            avgRight= (int) Core.mean(region1Right_Cb).val[0];



            //middle
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Middle_pointA, // First point which defines the rectangle
                    region1Middle_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
/*
            SkystoneAverageMiddle = avgMiddle;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avgMiddle > FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
            else if (avgMiddle > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
            else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Middle_pointA, // First point which defines the rectangle
                    region1Middle_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill





            //Left
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Left_pointA, // First point which defines the rectangle
                    region1Left_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines
/*
            SkystoneAverageLeft = avgLeft;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
                if(avgLeft> FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
                else if (avgLeft > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
                else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/
            Imgproc.rectangle(
                input, // Buffer to draw on
                region1Left_pointA, // First point which defines the rectangl
                region1Left_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill






            //Right
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Right_pointA, // First point which defines the rectangle
                    region1Right_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines

            SkystoneAverageRight = avgRight;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if((avgRight > avgLeft) && (avgRight > avgMiddle))
            {
                position = MarkerPosition.Right;
            }
            else if ((avgLeft > avgRight) && (avgLeft > avgMiddle))
            {
                position = MarkerPosition.Left;
            }
            else
            {
                position = MarkerPosition.Middle;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1Right_pointA, // First point which defines the rectangle
                    region1Right_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                                -1); // Negative thickness means solid fill



                    return input;
        }



        public int getAnalysisMiddle()
        {
            return avgMiddle;
        }

        public int getAnalysisLeft()
        {
            return avgLeft;
        }

        public int getAnalysisRight()
        {
            return avgRight;
        }
    }

    public void zeropower() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void StraifLeft(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        rightFront.setPower(-speed);
    }

    public void StraifRight(double speed) {
        leftFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        rightFront.setPower( speed);
    }
    public void allpower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void duckySpins(double power) {
        duckySpinner.setPower(power);
    }



}


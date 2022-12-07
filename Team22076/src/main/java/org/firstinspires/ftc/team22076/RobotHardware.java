package org.firstinspires.ftc.team22076;

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
 * DAMAGES (INCLOSSLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* I like spaggeti
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
 */
public class RobotHardware {

    LinearOpMode OpMode_;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor liftmotor;
    DcMotor swivelmotorRight;
    DcMotor swivelmotorLeft;
    Servo gripservo;

    BNO055IMU imu = null;      // Control/Expansion Hub IMU

    public SkystoneDeterminationPipeline pipeline;
    public WebcamName webcamLeft; // USB 3.0
    public WebcamName webcamRight; // USB 2.0
    public OpenCvSwitchableWebcam switchableWebcam;
    public int FinalMarkerPos;

    public enum cameraSelection {
        LEFT,
        RIGHT
    }
    /* ----------------------------------Constants ------------------------------------*/

    //--------For Motor drive straight------
    private final double COUNTS_PER_MOTOR_REV = 28;    //  REV-41-1291 Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 20;     //Gear Ratio
    private final double MOTOR_ENCODER_COUNT_PER_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private final double WHEEL_DIAMETER = 4.0;//4.3;
    private final double CORRECTION_VALUE = 1.1; //correction factor for system slop
    private final double INCHES_PER_REV = WHEEL_DIAMETER * Math.PI;  //Circumference of the wheel
    final double COUNTS_PER_INCH = MOTOR_ENCODER_COUNT_PER_REV / (INCHES_PER_REV * CORRECTION_VALUE); //  Counts/Inch
    //-----------for Non Encoder turn------------
    private final double CORRECTION_FACTOR = 180.0 / 165.0;
    private final double ROBOT_WIDTH_INCHES = 14.75; //outside of wheels 15.75,  (inside = 13.75)
    private final double ROBOT_LENGTH_HUB_TO_HUB = 10.5;  //Measurement from center of 1st wheel to center of 3rd wheel
    private final double DIAMETER_OF_ROBOT = Math.sqrt(Math.pow(ROBOT_WIDTH_INCHES, 2) + Math.pow(ROBOT_LENGTH_HUB_TO_HUB, 2));
    private final double ROBOT_TURN_CIRCUMFERENCE_INCHES = DIAMETER_OF_ROBOT * Math.PI;
    final double INCHES_PER_DEGREE = ROBOT_TURN_CIRCUMFERENCE_INCHES / 360 * CORRECTION_FACTOR;
    //final double COUNTS_PER_DEG = COUNTS_PER_INCH * INCHES_PER_DEGREE;

    //-----////////--For IMU---------------------
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    final double P_TURN_GAIN = 0.025;     // Larger is more responsive, but also less stable
    final double P_DRIVE_GAIN = 0.015;     // Larger is more responsive, but also less stable


    /* Constructor */
    public RobotHardware(LinearOpMode OpMode)
    {

        OpMode_ = OpMode;

        //Assign Motors
        leftFront = OpMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = OpMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = OpMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = OpMode.hardwareMap.get(DcMotor.class, "rightRear");

        liftmotor = OpMode.hardwareMap.get(DcMotor.class, "liftMotor");
        swivelmotorRight = OpMode.hardwareMap.get(DcMotor.class, "swivelMotorRight");
        swivelmotorLeft = OpMode.hardwareMap.get(DcMotor.class, "swivelMotorLeft");
        gripservo = OpMode.hardwareMap.get(Servo.class, "gripServo");


        //reverse the default rotation of the motor
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        swivelmotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set the motors to BRAKE mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swivelmotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swivelmotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot is stationary.  Reset the encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders by default
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swivelmotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swivelmotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //----------------IMU-----------------
        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
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

        /* ************************************************************************************
         *
         *              This section is for the web camera initoniazation
         *
         ************************************************************************************/
        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());

        //Only using one webcam
        webcamLeft = OpMode_.hardwareMap.get(WebcamName.class, "Webcam"); // USB 3.0
        // webcamRight = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new SkystoneDeterminationPipeline(320/2-70, 240-90);
        //webcam.setPipeline(pipeline);

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamLeft);
        //switchableWebcam.openCameraDevice();

        //OpMode_.sleep(1000);

        //switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        //switchableWebcam.setActiveCamera(webcamLeft);
        //final boolean usefront = true;

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired camera here

                //   if (camera == cameraSelection.LEFT) {
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                //switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                switchableWebcam.setActiveCamera(webcamLeft);
                //    } else {
                //       switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                //       switchableWebcam.setActiveCamera(webcamRight);
                //   }
            }

            @Override
            public void onError(int errorCode) {
                //print somethng if error???
            }
        });

        switchableWebcam.setPipeline(pipeline);
        /* ************************************************************************************
         *
         *              end of section for the web camera initoniazation
         *
         ************************************************************************************/
    } //end robot HW constructor


    /* ************************************************************************************
     *
     *              This section is for the web camera operation
     *
     ************************************************************************************/
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        //An enum to define the skystone position
        //one = red 2 = green 3 = blue
        public enum MarkerPos {
            ONE,
            TWO,
            THREE
        }
        //defaulting marker position to spot two
        public MarkerPos markerPos = MarkerPos.TWO;
        //public MarkerPos FinalMarkerPos;

        //Some color constants
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        //The core values which define the location and size of the sample regions
        //box location and dimensions
        //static final
        //Point REGION1_TOPLE
        // FT_ANCHOR_POINT = new Point(x,y); // 200, 165
        Point REGION1_TOPLEFT_ANCHOR_POINT;
        //Point REGION2_TOPLEFT_ANCHOR_POINT;
        //Point REGION3_TOPLEFT_ANCHOR_POINT;

        //18" inch away the cone is around these dimensions
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 75;

        //public final int FOUR_RING_THRESHOLD = 150;
        //public final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA;
        Point region1_pointB;

        //Point region2_pointA;
        //Point region2_pointB;

        //Point region3_pointA;
       //Point region3_pointB;

        //Working variables
        Mat region1_Cr;
        Mat region2_Cg;
        Mat region3_Cb;
        //Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cg = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.MarkerPos position = MarkerPos.TWO;
        //camers to be mounted to right side of robot marker cone is 18 inches form camera detection box y value will be
        // 120 and 1 half our bos with is 320 minus our marker with is our x value
        //public SkystoneDeterminationPipeline(int x, int y, int x2, int y2, int x3, int y3) {
        public SkystoneDeterminationPipeline(int x, int y) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            /*
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(x2, y2); // 300, 165
            region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            REGION3_TOPLEFT_ANCHOR_POINT = new Point(x3, y3); // 100, 165
            region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            */
        }



        //This function takes the RGB frame and pulls the red green blue colors
        void inputToCb(Mat input) {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Core.extractChannel(YCrCb, Cb, 2);
            Core.extractChannel(input, Cr, 0);
            Core.extractChannel(input, Cg, 1);
            Core.extractChannel(input, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cg = Cg.submat(new Rect(region1_pointA, region1_pointB));
            region3_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        //this detects color averages

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cg).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
            if (avg1 > avg2 && avg1 > avg3) {
                markerPos = MarkerPos.ONE;
            } else if (avg2 > avg1 && avg2 > avg3) {
                markerPos = MarkerPos.TWO;
            } else if (avg3 > avg1 && avg3 > avg2) {
                markerPos = MarkerPos.THREE;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            /*
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill


            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }
            else if (avg1 > ONE_RING_THRESHOLD)
            {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }
            else
            {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }
*/

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }  //end of web cam contructor

    /* ***********************************************************************************
     *
     *              End of Web Cam Section
     *
     ************************************************************************************/



} //end of class

package org.firstinspires.ftc.teamcode.Team2844.Drivers;

import android.graphics.Region;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class MandoRobotHardware
{
    LinearOpMode OpMode_;

    public DcMotor  leftFrontDrive;
    public DcMotor  leftBackDrive;
    public DcMotor  rightFrontDrive;
    public DcMotor  rightBackDrive;

    public Servo    wobbleServo;
    public Servo    clasper;
    public Servo    nucketyServo;
    public Servo    sweepyServo;

    public DcMotor  frontshot;
    public DcMotor  backshot;

    public DcMotor intake;

    public SkystoneDeterminationPipeline pipeline;
    public WebcamName webcamLeft; //
    public WebcamName webcamRight; //
    public OpenCvSwitchableWebcam switchableWebcam;

    // wobble goal arm servo
    // shooter motor 1st
    // shooter motor 2nd
    // box servo
    // ring pushing servo

    public BNO055IMU imu;

    private final double     COUNTS_PER_MOTOR_REV    = 28 ;    //  AndyMark Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 21.321;   // This is < 1.0 if geared UP
    private final double     ONE_MOTOR_COUNT         = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private final double     WHEEL_CIRCUMFERENCE     = 4 * (3.14159265);
    public final double      COUNTS_PER_INCH         = ONE_MOTOR_COUNT / WHEEL_CIRCUMFERENCE;  //TODO determine in class

    public enum cameraSelection
    {
        LEFT,
        RIGHT
    }

    /* Constructor */
    public MandoRobotHardware(LinearOpMode opMode, int x, int y, final cameraSelection camera)
    {
        /* Public OpMode members */
        OpMode_ = opMode;

        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());
        webcamLeft = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Left"); // USB 3.0
        webcamRight = OpMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new SkystoneDeterminationPipeline(x, y);
        //webcam.setPipeline(pipeline);

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamRight);
        switchableWebcam.openCameraDevice();


        //OpMode_.sleep(1000);

        switchableWebcam.setPipeline(pipeline);

        //switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        //switchableWebcam.setActiveCamera(webcamFront);
        //final boolean usefront = true;

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //pick desired camera here

                if (camera == cameraSelection.LEFT)
                {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(webcamLeft);
                }
                else
                {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcamRight);
                }
            }
        });


        // Define and Initialize Motors
        leftFrontDrive = OpMode_.hardwareMap.get(DcMotor.class, "lfmotor"); // ch motor 2
        leftBackDrive = OpMode_.hardwareMap.get(DcMotor.class, "lbmotor"); // ch motor 3
        rightFrontDrive = OpMode_.hardwareMap.get(DcMotor.class, "rfmotor"); // ch motor 0
        rightBackDrive = OpMode_.hardwareMap.get(DcMotor.class, "rbmotor"); // ch motor 1

        wobbleServo = OpMode_.hardwareMap.get(Servo.class, "wobble"); // ch servo 3
        clasper = OpMode_.hardwareMap.get(Servo.class, "clasper"); // ch servo 2
        nucketyServo = OpMode_.hardwareMap.get(Servo.class, "nuckety"); // ch servo 1
        sweepyServo = OpMode_.hardwareMap.get(Servo.class, "sweepy"); // ch servo 0

        frontshot = OpMode_.hardwareMap.get(DcMotor.class, "fshot"); // eh motor 0
        backshot = OpMode_.hardwareMap.get(DcMotor.class, "bshot"); // eh motor 1

        intake = OpMode_.hardwareMap.get(DcMotor.class, "intake"); // eh motor 2

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run without encoders by default
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        //An enum to define the skystone position
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions
        //box location and dimensions
        //static final
        //Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x,y); // 200, 165
        Point REGION1_TOPLEFT_ANCHOR_POINT;

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 45;

        public final int  FOUR_RING_THRESHOLD = 150;
        public final int  ONE_RING_THRESHOLD = 135;

        Point region1_pointA;
        Point region1_pointB;

        //Working variables
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.RingPosition position = SkystoneDeterminationPipeline.RingPosition.FOUR;

        public SkystoneDeterminationPipeline(int x, int y)
        {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }

        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

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

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
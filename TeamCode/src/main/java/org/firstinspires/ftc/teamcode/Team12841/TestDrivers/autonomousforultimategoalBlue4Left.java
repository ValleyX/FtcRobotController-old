package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive4motors;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware4motors;
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

import java.util.Locale;

/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
//package org.firstinspires.ftc.teamcode.vision;
//import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Test: autonomous for ultimate goal blue4 left", group="Test")
public class autonomousforultimategoalBlue4Left extends LinearOpMode {
    public SkystoneDeterminationPipeline pipeline;
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;

    public enum cameraSelection
    {
        LEFTCAM,
        RIGHTCAM
    }



    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(125, 135);

        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
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

        public int getAnalysis() {
            return avg1;
        }
    }


    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    RobotHardware4motors robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        {
            final autonomousforultimategoalBlue4Left.cameraSelection camera = cameraSelection.LEFTCAM;

            //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

            webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
            //switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam1);

            switchableWebcam.openCameraDevice();
            pipeline = new SkystoneDeterminationPipeline();
            switchableWebcam.setPipeline(pipeline);
            switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    //pick desired webcam here

                    if (camera == cameraSelection.RIGHTCAM)
                    {
                        switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                        switchableWebcam.setActiveCamera(webcam1);
                    }
                    else
                    {
                        switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                        switchableWebcam.setActiveCamera(webcam2);
                    }
                }

            });

            autonomousforultimategoalBlue4Left.SkystoneDeterminationPipeline.RingPosition path = pipeline.position;

            while (!isStarted())
            {
                path = pipeline.position;
                telemetry.addData("Number of Rings", path);
                telemetry.update();
            }

            waitForStart();
/*
            while (opModeIsActive()) {
                telemetry.addLine("PRESS A/B TO SWITCH CAMERA\n");
                telemetry.addData("Frame Count", switchableWebcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", switchableWebcam.getFps()));
                telemetry.addData("Total frame time ms", switchableWebcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", switchableWebcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", switchableWebcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", switchableWebcam.getCurrentPipelineMaxFps());
                telemetry.update();

                telemetry.addData("position", pipeline.position);
                //
                // To switch the active camera, simply call
                 // {@link OpenCvSwitchableWebcam#setActiveCamera(WebcamName)}
                 //
                if (gamepad1.a) {
                    switchableWebcam.setActiveCamera(webcam1);
                } else if (gamepad1.b) {
                    switchableWebcam.setActiveCamera(webcam1);
                }

                sleep(100);
            }
            */
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        robot = new RobotHardware4motors(hardwareMap, this);
        EncoderDrive4motors encoder = new EncoderDrive4motors(robot);
        double heading;
        waitForStart();

        final double TopGoalPower = 0.67;
        final double DriveUpInches = 48;
        final double WhitelineInches = 67;

        encoder.StartAction(1, DriveUpInches, DriveUpInches, 5, true); // drive up to shooting line
        encoder.StartAction(1, 1, -1, 5, true); // turn one inch right

        for (int i = 0; i < 3; i++) {

            robot.shooterfront.setPower(TopGoalPower);
            robot.shooterback.setPower(TopGoalPower);

            sleep(200);

            robot.ringpusher.setPosition(robot.backringpusher_POS);
            sleep(700);
            robot.ringpusher.setPosition(robot.frontringpusher_POS);
            sleep(950);

            robot.shooterfront.setPower(0);
            robot.shooterback.setPower(0);

            sleep(2000);
        }

        encoder.StartAction(1, -1, 1, 5, true); // turn one inch left


//Kinda blue box a (left start)
        if (pipeline.position == autonomousforultimategoalBlue4Left.SkystoneDeterminationPipeline.RingPosition.NONE) {
            encoder.StartAction(1, 65 - DriveUpInches, 65- DriveUpInches, 5, true);

            robot.ArmsSetWabbleGoal();

            robot.turntoheading(1,-5);

        }
//kinda blue box b (left start)
            if (pipeline.position == autonomousforultimategoalBlue4Left.SkystoneDeterminationPipeline.RingPosition.ONE) {
                encoder.StartAction(1, 99, 99, 5, true);

                robot.turntoheading(1,90);

                robot.turntoheading(1,0);

                encoder.StartAction(1, -30, -30, 5, true);

            }
//kinda blue box c (left start)
            if (pipeline.position == autonomousforultimategoalBlue4Left.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                encoder.StartAction(1, 102, 102, 5, true);

                robot.ArmsSetWabbleGoal();

                encoder.StartAction(1, -30, -30, 5, true);
            }
        }
        double formatAngle (AngleUnit angleUnit,double angle){
            double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
            double normalDegrees = AngleUnit.DEGREES.normalize(degrees);

            return normalDegrees;
            //double heading =  formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees ( double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }

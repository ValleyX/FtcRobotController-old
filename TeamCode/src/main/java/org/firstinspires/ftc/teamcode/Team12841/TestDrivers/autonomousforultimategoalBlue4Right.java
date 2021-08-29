package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive4motors;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;
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

@Autonomous(name="blue4 right", group="Test")
//@Disabled
public class autonomousforultimategoalBlue4Right extends LinearOpMode {
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
         * An enum to define the Ringposition
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 135);

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 40;

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
    RobotHardware4motors robot;
    SkystoneDeterminationPipeline.RingPosition path = SkystoneDeterminationPipeline.RingPosition.FOUR;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware4motors(hardwareMap, this);
        EncoderDrive4motors encoder = new EncoderDrive4motors(robot);

        final cameraSelection camera = cameraSelection.RIGHTCAM;

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);

        switchableWebcam.openCameraDevice();
        pipeline = new SkystoneDeterminationPipeline();
        switchableWebcam.setPipeline(pipeline);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired webcam here

                if (camera == cameraSelection.RIGHTCAM) {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcam1);
                } else {
                    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                    switchableWebcam.setActiveCamera(webcam2);
                }
            }

        });


        robot.Servoarm.setPosition(robot.ARMUP_POS);
        robot.Servohand.setPosition(robot.HANDCLOSE_POS); //The close on hand

        while (!isStarted()) {
            path = pipeline.position;
            telemetry.addData("Number of Rings", path);
            telemetry.update();
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


        double heading;
        final double TopGoalPower = 0.67;
        final double DriveUpInches = 28;
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

        //kinda blue box A (right start)
        if (path == SkystoneDeterminationPipeline.RingPosition.NONE) {


            encoder.StartAction(1, 65 - DriveUpInches, 65 - DriveUpInches, 5, true);

            robot.turntoheading(1,-90);
            /*robot.turnleft(1);

            //these 2 lines get heading from IMU
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);

            System.out.println("ValleyX: " + heading);
            while (heading <= 90) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX: " + heading);
            }
            robot.power0drive();*/

            encoder.StartAction(1, 10, 10, 5, true); // move to box a

            robot.ArmsSetWabbleGoal();

            encoder.StartAction(1, -10, -10, 5, true); // move back for teammate
        }

        //kinda blue box B (right start)
        if (path == SkystoneDeterminationPipeline.RingPosition.ONE) {
            encoder.StartAction(1, 72 - DriveUpInches, 72 - DriveUpInches, 5, true); // originally 77
            encoder.StartAction(1, -14, 14, 5, true); // turn left to get wobbble goal into box

            robot.ArmsSetWabbleGoal();

            encoder.StartAction(1, 14, -14, 5, true); // turn right to get robot straight
            encoder.StartAction(1, -7, -7, 5, true); // back up to white line

        }
    /*
    //kinda blue box C (right start)
     */
        if (path == SkystoneDeterminationPipeline.RingPosition.FOUR) {
            encoder.StartAction(1, 105 - DriveUpInches, 105 - DriveUpInches, 5, true); // 110 was original

            robot.turntoheading(1,-90);
            /*robot.turnleft(1);

            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX left: " + heading);
            while (heading <= 90) {
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                System.out.println("ValleyX left: " + heading);
            }
            robot.power0drive();*/

            encoder.StartAction(1, 10, 10, 5, true); // drive to box c

            robot.ArmsSetWabbleGoal();

            encoder.StartAction(1, -14, -14, 5, true); // drive back from box c

            robot.turntoheading(1,-1);
            /*robot.turnright(1);

            System.out.println("ValleyX after power:");

            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            System.out.println("ValleyX right: " + heading);
            while (heading >= 1) {
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                System.out.println("ValleyX right: " + heading);
            }
            robot.power0drive();*/

            encoder.StartAction(1, -45, -45, 5, true); // back to white line
        }

    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        double normalDegrees = AngleUnit.DEGREES.normalize(degrees);

        return -normalDegrees;
        //double heading =  formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

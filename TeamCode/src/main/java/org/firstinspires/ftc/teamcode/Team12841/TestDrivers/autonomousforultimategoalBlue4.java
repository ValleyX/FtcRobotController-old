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

@Autonomous(name="Test: autonomous for ultimate goal blue4", group="Test")
public class autonomousforultimategoalBlue4 extends LinearOpMode {
    public SkystoneDeterminationPipeline pipeline;
    WebcamName webcam1;
    // WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;




/*
class SamplePipeline extends OpenCvPipeline
{
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);

        return input;
    }
}
*/

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
    public void runOpMode() throws InterruptedException {


        //    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
        {

            //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

            webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            // webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //   switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
            switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam1);

            switchableWebcam.openCameraDevice();
            pipeline = new SkystoneDeterminationPipeline();
            switchableWebcam.setPipeline(pipeline);
            switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

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


        //RobotHardware robot = new RobotHardware(hardwareMap, this);
        robot = new RobotHardware4motors(hardwareMap, this);
        EncoderDrive4motors encoder = new EncoderDrive4motors(robot);
        double heading;
        waitForStart();

        final double fullturn = 3.14 * 18; //18 inches
        final double halfturn = 3.14 * 9; // 9 inches
        final double quarterturn = 3.14 * 4.5; //4.5 inches


        //kinda blue box A (right start)
        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
            encoder.StartAction(1, 75, 75, 30, true);

            //turn left 90 degress
            //robot.leftDrive.setPower(-0.80);
            //robot.rightDrive.setPower(0.80);
            robot.leftDrivefront.setPower(-0.80);
            robot.leftDriveback.setPower(-0.80);
            robot.rightDrivefront.setPower(0.80);
            robot.rightDriveback.setPower(0.80);

            //these 2 lines get heading from IMU
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);

            System.out.println("ValleyX: " + heading);
            while (heading <= 89) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //heading = formatAngle(angles.angleUnit, angles.firstAngle);
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX: " + heading);
            }
            //robot.rightDrive.setPower(0);
            //robot.leftDrive.setPower(0);
            robot.leftDrivefront.setPower(0);
            robot.leftDriveback.setPower(0);
            robot.rightDrivefront.setPower(0);
            robot.rightDriveback.setPower(0);

            encoder.StartAction(1, 22, 22, 30, true);
            encoder.StartAction(1, -22, -22, 30, true);
        }

    //kinda blue box B (right start)
    if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
        encoder.StartAction(1, 97, 97, 30, true);

        //robot.leftDrive.setPower(-0.80);
        //robot.rightDrive.setPower(0.80);
        robot.leftDrivefront.setPower(-0.80);
        robot.leftDriveback.setPower(-0.80);
        robot.rightDrivefront.setPower(0.80);
        robot.rightDriveback.setPower(0.80);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);

        System.out.println("ValleyX: " + heading);
        while (heading <= 89) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);
            System.out.println("ValleyX: " + heading);
        }
        //robot.rightDrive.setPower(0);
        //robot.leftDrive.setPower(0);
        robot.leftDrivefront.setPower(0);
        robot.leftDriveback.setPower(0);
        robot.rightDrivefront.setPower(0);
        robot.rightDriveback.setPower(0);

        sleep(5000);

        //robot.leftDrive.setPower(0.80);
        //robot.rightDrive.setPower(-0.80);
        robot.leftDrivefront.setPower(0.80);
        robot.leftDriveback.setPower(0.80);
        robot.rightDrivefront.setPower(-0.80);
        robot.rightDriveback.setPower(-0.80);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);

        System.out.println("ValleyX: " + heading);
        while (heading >= 0) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.firstAngle);
            System.out.println("ValleyX: " + heading);
        }

        //robot.rightDrive.setPower(0);
        //robot.rightDrive.setPower(0);
        robot.leftDrivefront.setPower(0);
        robot.leftDriveback.setPower(0);
        robot.rightDrivefront.setPower(0);
        robot.rightDriveback.setPower(0);
        encoder.StartAction(1, -25, -25, 10, true);
    }
    /*
    //kinda blue box C (right start)

     */
    if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
        encoder.StartAction(0.5, 123, 123, 30, true);
        robot.leftDrivefront.setPower(-0.15);
        robot.leftDriveback.setPower(-0.15);
        robot.rightDrivefront.setPower(0.15);
        robot.rightDriveback.setPower(0.15);

        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = formatAngle(angles.angleUnit, angles.firstAngle);
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        System.out.println("ValleyX left: " + heading);
        while (heading <= 89) {
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX left: " + heading);
        }
        robot.leftDrivefront.setPower(0);
        robot.leftDriveback.setPower(0);
        robot.rightDrivefront.setPower(0);
        robot.rightDriveback.setPower(0);

        encoder.StartAction(0.5, 25, 25, 30, true);

        encoder.StartAction(0.5, -25, -25, 30, true);

        // robot.leftDrivefront.setPower(0);
        //robot.rightDrive.setPower(0);

        System.out.println("ValleyX : before sleep");
        sleep(3000);
        System.out.println("ValleyX : After sleep");

        robot.leftDrivefront.setPower(0.15);
        robot.leftDriveback.setPower(0.15);
        robot.rightDrivefront.setPower(0.15);
        robot.rightDriveback.setPower(-0.15);
        System.out.println("ValleyX after power:");
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = formatAngle(angles.angleUnit, angles.firstAngle);
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        System.out.println("ValleyX right: " + heading);
        while (heading >= 0) {
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX right: " + heading);
        }
        robot.leftDrivefront.setPower(0);
        robot.leftDriveback.setPower(0);
        robot.rightDrivefront.setPower(0);
        robot.rightDriveback.setPower(0);
        //robot.leftDrive.setPower(0);
        //robot.rightDrive.setPower(0);
        encoder.StartAction(0.5, -54, -54, 30, true);
    }
    /*
//Kinda blue box a (left start)
    encoder.StartAction(0.5,99,99,30,true);
    robot.leftDrive.setPower(0.15);
    robot.rightDrive.setPower(-0.15);
        robot.leftDrivefront.setPower(0.15);
        robot.leftDriveback.setPower(0.15);
        robot.rightDrivefront.setPower(-0.15);
        robot.rightDriveback.setPower(-0.15);

    heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    System.out.println("ValleyX left: " + heading);
    while (heading >= -178){
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        System.out.println("ValleyX left: " + heading);
    }
    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
     robot.leftDrivefront.setPower(0);
     robot.leftDriveback.setPower(0);
     robot.rightDriveback.setPower(0);
     robot.rightDriveback.setPower(0);

//kinda blue box b (left start)
    encoder.StartAction(1,99,99,30,true);
    robot.leftDrive.setPower(0.2);
    robot.rightDrive.setPower(-0.2);
        robot.leftDrivefront.setPower(0.2);
        robot.leftDriveback.setPower(0.2);
        robot.rightDrivefront.setPower(-0.2);
        robot.rightDriveback.setPower(-0.2);

    heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    System.out.println("ValleyX left: " + heading);
    while (heading > -90){
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        System.out.println("ValleyX left: " + heading);
    }
    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
     robot.leftDrivefront.setPower(0);
     robot.leftDriveback.setPower(0);
     robot.rightDrivefront.setPower(0);
     robot.rightDriveback.setPower(0);

    robot.leftDrive.setPower(-0.2);
    robot.rightDrive.setPower(0.2);
        robot.leftDrivefront.setPower(-0.2);
        robot.leftDriveback.setPower(-0.2);
        robot.rightDrivefront.setPower(0.2);
        robot.rightDriveback.setPower(0.2);

    heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    System.out.println("ValleyX left: " + heading);
    while (heading < 0){
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        System.out.println("ValleyX left: " + heading);
    }
    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
    robot.leftDrivefront.setPower(0);
     robot.leftDriveback.setPower(0);
     robot.rightDrivefront.setPower(0);
     robot.rightDriveback.setPower(0);





    encoder.StartAction(1,-30,-30,10,true);

//kinda blue box c (left start)
    encoder.StartAction(1,102,102,30,true);
    encoder.StartAction(1,-30,-30,20,true);

*/
    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        double normalDegrees = AngleUnit.DEGREES.normalize(degrees);

        return normalDegrees;
        //double heading =  formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

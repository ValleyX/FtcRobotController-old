
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

@Autonomous(name="Test: autonomous for ultimate goal red4left ", group="Test")
@Disabled

public class autonomousforultimategoalRed4left extends LinearOpMode {
    public autonomousforultimategoalRed4left.SkystoneDeterminationPipeline pipeline;
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;

    public enum cameraSelection {
        LEFTCAM,
        RIGHTCAM
    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        //
        //An enum to define the skystone position
        //
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        //
        // Some color constants
        //
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //
        // The core values which define the location and size of the sample regions
        //
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

        //
        // Working variables
        //
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition position = autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.FOUR;

        //
        // This function takes the RGB frame, converts to YCrCb,
        // and extracts the Cb channel to the 'Cb' variable
        //
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

            position = autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.NONE;
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

    @Override
    public void runOpMode() throws InterruptedException {

        final autonomousforultimategoalRed4left.cameraSelection camera = autonomousforultimategoalRed4left.cameraSelection.LEFTCAM;

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
        //switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam1);

        switchableWebcam.openCameraDevice();
        pipeline = new autonomousforultimategoalRed4left.SkystoneDeterminationPipeline();
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

        autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition path = pipeline.position;

        while (!isStarted()) {
            path = pipeline.position;
            telemetry.addData("Number of Rings", path);
            telemetry.update();
        }

        waitForStart();
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


        RobotHardware4motors robot = new RobotHardware4motors(hardwareMap, this);
        EncoderDrive4motors encoder = new EncoderDrive4motors(robot);
        double heading;
        waitForStart();

        final double fullturn = 3.14 * 18; //18 inches
        final double halfturn = 3.14 * 9; // 9 inches
        final double quarterturn = 3.14 * 4.5; //4.5 inches

        //kinda red box A (left start)
        if (pipeline.position == autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.NONE) {
            encoder.StartAction(1, 75, 75, 5, true);

            robot.turntoheading(1,89);
            /*//turn left 90 degress
            robot.leftDrivefront.setPower(0.80);
            robot.leftDriveback.setPower(0.80);
            robot.rightDrivefront.setPower(-0.80);
            robot.rightDriveback.setPower(-0.80);

            //these 2 lines get heading from IMU
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            System.out.println("ValleyX: " + heading);
            while (heading >= -89) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //heading = formatAngle(angles.angleUnit, angles.firstAngle);
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX: " + heading);
            }

            robot.power0drive();*/

            encoder.StartAction(1, 22, 22, 5, true);
            encoder.StartAction(1, -22, -22, 5, true);
        }

        //kinda red box C(left start)
        if (pipeline.position == autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            encoder.StartAction(0.5, 123, 123, 5, true);

            robot.turntoheading(1,89);
            /*robot.leftDrivefront.setPower(0.15);
            robot.leftDriveback.setPower(0.15);
            robot.rightDrivefront.setPower(-0.15);
            robot.rightDriveback.setPower(-0.15);

            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            System.out.println("ValleyX left: " + heading);
            while (heading >= -89) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //heading = formatAngle(angles.angleUnit, angles.firstAngle);
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX left: " + heading);
            }
            robot.power0drive();*/

            encoder.StartAction(0.5, 25, 25, 5, true);

            encoder.StartAction(0.5, -25, -25, 5, true);

            robot.power0drive();

            System.out.println("ValleyX : before sleep");
            sleep(3000);
            System.out.println("ValleyX : After sleep");

            robot.turntoheading(1,0);
            /*robot.leftDrivefront.setPower(-0.15);
            robot.leftDriveback.setPower(-0.15);
            robot.rightDrivefront.setPower(0.15);
            robot.rightDriveback.setPower(0.15);
            System.out.println("ValleyX after power:");
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //heading = formatAngle(angles.angleUnit, angles.firstAngle);
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            System.out.println("ValleyX right: " + heading);
            while (heading <= 0) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //heading = formatAngle(angles.angleUnit, angles.firstAngle);
                heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                System.out.println("ValleyX right: " + heading);
            }
            robot.power0drive();*/

            encoder.StartAction(0.5, -54, -54, 5, true);

            //kinda blue box b (left start)
            if (pipeline.position == autonomousforultimategoalRed4left.SkystoneDeterminationPipeline.RingPosition.ONE){
                encoder.StartAction(1, 102, 102, 5, true);
            encoder.StartAction(1, -30, -30, 20, true);


                }
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



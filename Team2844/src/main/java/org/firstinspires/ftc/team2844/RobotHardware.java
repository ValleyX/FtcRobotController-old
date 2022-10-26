package org.firstinspires.ftc.team2844;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

public class RobotHardware {
    public LinearOpMode OpMode_;
    public WebcamName camCam;
    public PowerPlayPipeline pipeline;
    public OpenCvSwitchableWebcam switchableWebcam;

    //motors
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    //odometers
    public DcMotor encoderRight;
    public DcMotor encoderLeft;
    public DcMotor encoderAux;

    //IMU
    public BNO055IMU imu;


    //encoder
    public final double COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    public final double ONE_MOTOR_COUNT = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public final double Distance_in_one_rev = 4.0 * Math.PI; //in
    public final double COUNTS_PER_INCH = ONE_MOTOR_COUNT / Distance_in_one_rev;  //TODO determine// in class


    RobotHardware(LinearOpMode opMode) {
        OpMode_ = opMode;

        //remove motor encoders
        leftFront = OpMode_.hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack = OpMode_.hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = OpMode_.hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack = OpMode_.hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        imu = OpMode_.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        camCam = OpMode_.hardwareMap.get(WebcamName.class, "Webcamcolor");
        pipeline = new RobotHardware.PowerPlayPipeline(50, 50);
        int cameraMonitorViewId = OpMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpMode_.hardwareMap.appContext.getPackageName());

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, camCam, camCam);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //pick desired camera here
                if (true) {

                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    switchableWebcam.setActiveCamera(camCam);
                } else {
                    switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    //switchableWebcam.setActiveCamera(webcamRight);
                    switchableWebcam.setActiveCamera(camCam);
                }


            }


            @Override
            public void onError(int errorCode) {
                ///
                //* This will be called if the camera could not be opened
                //
            }
        });

        switchableWebcam.setPipeline(pipeline);

    }


    public static class PowerPlayPipeline extends OpenCvPipeline {
        //An enum to define the ring stack size
        public enum MarkerPosition {
            Red,
            Green,
            Blue
        }

        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        //Team Color Blue = 109 Average

        //The core values which define the location and size of the sample regions
        //box location and dimensions

        //Point REGION1_TOPLEFT_ANCHOR_POINT;

        Point REGION1_TOPLEFT_ANCHOR_POINT;


        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        //public final int  FOUR_RING_THRESHOLD = 150;

        //public final int  ONE_RING_THRESHOLD = 135;


        Point region1Left_pointA;
        Point region1Left_pointB;


        //Working variables
        Mat region1Middle_Cb;
        Mat region1Left_Cb;
        Mat region1Left_Cr;
        Mat region1Right_Cb;

        Mat region1Left_R;
        Mat region1Left_G;
        Mat region1Left_B;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat Cr = new Mat();

        Mat R = new Mat();
        Mat G = new Mat();
        Mat B = new Mat();


        int avgLeftBlue;
        int avgLeftRed;

        int avgLeftR;
        int avgLeftG;
        int avgLeftB;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile PowerPlayPipeline.MarkerPosition color = PowerPlayPipeline.MarkerPosition.Green;
        public volatile int SkystoneAverageMiddle;
        public volatile int SkystoneAverageLeft;
        public volatile int SkystoneAverageRight;


        public PowerPlayPipeline(int x, int y) {
            /*
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
             */

            REGION1_TOPLEFT_ANCHOR_POINT = new Point(200, 165); // 200, 165
            region1Left_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1Left_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        }

        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable

        void inputToCb(Mat input) {
            Core.extractChannel(input, R, 0);
            Core.extractChannel(input, G, 1);
            Core.extractChannel(input, B, 2);

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Core.extractChannel(YCrCb, Cr, 1);

        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1Left_Cb = Cb.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Left_Cr = Cr.submat(new Rect(region1Left_pointA, region1Left_pointB));

            region1Left_R = R.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Left_G = G.submat(new Rect(region1Left_pointA, region1Left_pointB));
            region1Left_B = B.submat(new Rect(region1Left_pointA, region1Left_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avgLeftBlue = (int) Core.mean(region1Left_Cb).val[0];
            avgLeftRed = (int) Core.mean(region1Left_Cr).val[0];

            avgLeftR = (int) Core.mean(region1Left_R).val[0];
            avgLeftG = (int) Core.mean(region1Left_G).val[0];
            avgLeftB = (int) Core.mean(region1Left_B).val[0];

            //middle
            // Thickness of the rectangle lines
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
            // Negative thickness means solid fill


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

            SkystoneAverageRight = avgLeftBlue;
            //position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis

            if ((avgLeftR > avgLeftB && avgLeftR > avgLeftG)) {
                color = PowerPlayPipeline.MarkerPosition.Red;
            } else if ((avgLeftG > avgLeftR && avgLeftG > avgLeftB)) {
                color = PowerPlayPipeline.MarkerPosition.Green;
            } else if ((avgLeftB > avgLeftR && avgLeftB > avgLeftG)) {
                color = PowerPlayPipeline.MarkerPosition.Blue;
            }


            //else
            //{
            //    position = SkystoneDeterminationPipeline.MarkerPosition.Green;
            //}

            // Negative thickness means solid fill


            return input;
        }


        public int getAnalysisLeft() {
            return avgLeftBlue;
        }

        public int getAnalysisRight() {
            return avgLeftRed;
        }
    }
    public void allpower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    //robot geometry constants (needs inputs)
    /*final static double L = ;   //distance between encoder 1 and 2 in cm
    final static double B = ;   //distance between the midpoint of encoder 1, 2, and 3
    final static double R = ;   //radius of the wheel
    final static double N = ;   //encoder ticks per revolution REV encoder
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    //keeping ytack of the od encoders between the tick updates?
    public int currentRightPos = 0;
    public int currentLeftPos = 0;
    public int currentAuxPos = 0;

    public int oldRightPos = 0;
    public int oldLeftPos = 0;
    public int oldAuxPos = 0;

    //find heading of the robot, h is heading, XyhVector is a tuple (x,y,h)
    public XyhVector  START_POS = new XyhVector(213,102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);


    public void odometry() {

        oldRightPos = currentRightPos;
        oldLeftPos = currentLeftPos;
        oldAuxPos = currentAuxPos;

        currentRightPos = -encoderRight.getCurrentPosition();
        currentLeftPos = -encoderLeft.getCurrentPosition();
        currentAuxPos = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPos - oldLeftPos;
        int dn2 = currentRightPos - oldRightPos;
        int dn3 = currentAuxPos - oldAuxPos;

        //the robot has changed position and heading a wee bit between 2 measurements
        double dtheta = cm_per_tick * (dn2-dn1) / L;
        double dx = cm_per_tick * (dn1+dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);

        //small movement of the robot gets added to the field coord system
        double theta = pos.h = (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;

        }*/



}

package org.firstinspires.ftc.team12841.drivers;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotHardware {

    //constants
    public final double countsPerMotorRev = 28.0;
    public final double driveGearReduction = 20.0;
    public final double motorEncoderCountsPerRev = countsPerMotorRev * driveGearReduction;
    public final double diameterOfWheel = 4.5;
    public final double circumferenceOfWheel = diameterOfWheel * Math.PI;
    public final double distancePerRev = motorEncoderCountsPerRev / circumferenceOfWheel;

    public final double liftCountsPerMotorRev = 28.0;
    public final double liftDriveGearReduction = 10.0;
    public final double liftMotorEncoderCountsPerRev = liftCountsPerMotorRev * liftDriveGearReduction;
    public final double liftDiameterOfWheel = 2.0;
    public final double liftCircumferenceOfWheel = liftDiameterOfWheel * Math.PI;
    public final double liftDistancePerRev = liftMotorEncoderCountsPerRev / liftCircumferenceOfWheel;
    // -----------------------Constance fo IMU Drive-------------------
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
//----------------------------------------------- end IMU ------------------------------

   public static final double openPinch = 0.8;
   public static final double closedPinch = 0.35;

    //Declare Motors/Servos
    public DcMotor lFMotor;
    public DcMotor lBMotor;
    public DcMotor rFMotor;
    public DcMotor rBMotor;
    public DcMotor liftMotor;
    public Servo pinch;

    public LinearOpMode opMode_;
    public BNO055IMU       imu         = null;      // Control/Expansion Hub IMU
   // public BHI260IMU       imu         = null;      // Control/Expansion Hub IMU
    public SkystoneDeterminationPipeline pipeline;
    //public WebcamName webcamLeft; // USB 3.0
    //public WebcamName webcamRight; // USB 2.0
    OpenCvWebcam webcamLeft;
//    public OpenCvSwitchableWebcam switchableWebcam;

    public enum cameraSelection {
        LEFT,
        RIGHT
    }

    public RobotHardware(LinearOpMode opmode)
    {
        opMode_=opmode;
/*
        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = opMode_.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
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
        imu = opMode_.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!opMode_.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode_.sleep(50);
            //System.out.println("ValleyX: Gyro sleeping");
            opMode_.idle();
        }

        if (!imu.isGyroCalibrated()) {
            System.out.println("ValleyX: Gyro not calibrated");
            opMode_.telemetry.addData("gyro", "not calibrated");
        }
        else
        {
            System.out.println("ValleyX: Gyro calibrated");
            opMode_.telemetry.addData("gyro", "calibrated");
        }

        opMode_.telemetry.update();
      /*  BHI260IMU. Parameters parameters = new BHI260IMU.Parameters();
        parameters.angleUnit            = BHI260IMU.AngleUnit.DEGREES;*/
      //  imu = opMode_.hardwareMap.get(BHI260IMU.class, "imu");
        //imu.initialize(parameters);

        //Hardware Map
        lFMotor = opMode_.hardwareMap.get(DcMotor.class, "lFMotor");
        lBMotor = opMode_.hardwareMap.get(DcMotor.class, "lBMotor");
        rFMotor = opMode_.hardwareMap.get(DcMotor.class, "rFMotor");
        rBMotor = opMode_.hardwareMap.get(DcMotor.class, "rBMotor");
        liftMotor = opMode_.hardwareMap.get(DcMotor.class, "liftMotor");
        pinch = opMode_.hardwareMap.get(Servo.class, "pinch");


        //invert motors
        lFMotor.setDirection(DcMotor.Direction.FORWARD);
        lBMotor.setDirection(DcMotor.Direction.FORWARD);
        rFMotor.setDirection(DcMotor.Direction.REVERSE);
        rBMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        lFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int cameraMonitorViewId = opMode_.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode_.hardwareMap.appContext.getPackageName());

       // webcamLeft = opMode_.hardwareMap.get(WebcamName.class, "Webcam"); // USB 3.0
        webcamLeft = OpenCvCameraFactory.getInstance().createWebcam(opMode_.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


        //webcamRight = opMode_.hardwareMap.get(WebcamName.class, "Webcam Right"); // USB 2.0
        pipeline = new SkystoneDeterminationPipeline(70, 220);

        //switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamLeft, webcamLeft);



       // switchableWebcam.openCameraDevice();


//        opMode_.sleep(1000);

       //switchableWebcam.setPipeline(pipeline);
       webcamLeft.setPipeline(pipeline);
      // webcamLeft.setPipeline(new SamplePipeline());

        //switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        //switchableWebcam.setActiveCamera(webcamFront);
        //final boolean usefront = true;

        webcamLeft.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.

        webcamLeft.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //pick desired camera here

                //if (camera == cameraSelection.LEFT) {
                webcamLeft.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                   // switchableWebcam.setActiveCamera(webcamLeft);
                //} else {
                //    switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                //    switchableWebcam.setActiveCamera(webcamRight);
                //}
            }

            @Override
            public void onError(int errorCode) {

            }
        });
/*


*/
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        //An enum to define the skystone position
        public enum MarkerPos {
            Blue,
            Green,
            Red
        }

        public MarkerPos markerPos = MarkerPos.Blue;

        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions
        //box location and dimensions
        //static final
        //Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x,y); // 200, 165
        Point REGION1_TOPLEFT_ANCHOR_POINT;


        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 65;

        public final int FOUR_RING_THRESHOLD = 150;
        public final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA;
        Point region1_pointB;



        //Working variables
        Mat region1_Cb;
        Mat region1_Cg;
        Mat region1_Cr;


        Mat Cb = new Mat();
        Mat Cg = new Mat();
        Mat Cr = new Mat();
        int avgBlue;
        int avgGreen;
        int avgRed;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.MarkerPos position = MarkerPos.Blue;

        public SkystoneDeterminationPipeline(int x, int y) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y); // 200, 165
            region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        }


        //This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(input, Cb, 2);
            Core.extractChannel(input, Cg, 1);
            Core.extractChannel(input, Cr, 0);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cg = Cg.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));


        }

        @Override
        public Mat processFrame(Mat input) {




            inputToCb(input);

            avgBlue = (int) Core.mean(region1_Cb).val[0];
            avgGreen = (int) Core.mean(region1_Cg).val[0];
            avgRed = (int) Core.mean(region1_Cr).val[0];




            if (avgBlue > avgGreen && avgBlue > avgRed){
                markerPos = MarkerPos.Blue;
            } else if (avgGreen > avgBlue && avgGreen > avgRed){
                markerPos = MarkerPos.Green;
            } else if (avgRed > avgBlue && avgRed > avgGreen){
                markerPos = MarkerPos.Red;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill



            return input;
        }
        public int getAnalysis() {
            return avgBlue;
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
          `   * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                //webcam.pauseViewport();
                webcamLeft.pauseViewport();
            }
            else
            {
              //  webcam.resumeViewport();
                webcamLeft.resumeViewport();
            }
        }
    }

}
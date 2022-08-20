package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team2844.Drivers.DistanceDriverTest;
import org.firstinspires.ftc.team2844.Drivers.EncoderDriveMecha;
import org.firstinspires.ftc.team2844.Drivers.LiftDriverTest;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.team2844.dogecv.filters.LeviColorFilter;

//@Disabled

@TeleOp(name="Testbot")
public class TestBot extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, this,320,240, RobotHardware.cameraSelection.UP);

        EncoderDriveMecha encodermecha = new EncoderDriveMecha(robot);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        DistanceDriverTest Driveto = new DistanceDriverTest(robot, headingdrive);
        LiftDriverTest liftto = new LiftDriverTest(robot);
        //SensorREVColorDistance_CB colorto  = new SensorREVColorDistance_CB(robot, headingdrive);


        double motorspeed = 0.4;
        ElapsedTime elapsedTime = new ElapsedTime();

        System.out.println("valleyx: im here4");

        Boolean rbpressed = false;
        Boolean lbpressed = false;


        while (!isStarted()) {

            if (gamepad1.a)
            {
               // robot.switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
               // robot.switchableWebcam.stopStreaming();
               // robot.switchableWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                robot.switchableWebcam.setActiveCamera(robot.WebcamUp);
            }

            if (gamepad1.b)
            {
                liftto.LiftToDistance(1,12,true);

            }

            if (gamepad1.dpad_right){
                robot.switchableWebcam.setPipeline(robot.redPipeline);
            }

            if (gamepad1.dpad_left) {
                robot.switchableWebcam.setPipeline(robot.goldPipeline);
            }

            if (gamepad1.dpad_down){
                robot.switchableWebcam.setPipeline(robot.bluePipeline);
            }


                if ((gamepad1.y) && (!rbpressed)) {
                    robot.redPipeline.redTheshold += 1;
                    robot.redPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robot.redPipeline.redTheshold);
                    rbpressed = true;
                } else if (!gamepad1.y) {
                    rbpressed = false;
                }

                if ((gamepad1.x)  && (!lbpressed)) {
                    robot.redPipeline.redTheshold -= 1;
                    robot.redPipeline.redFilter.updateSettings(LeviColorFilter.ColorPreset.RED,robot.redPipeline.redTheshold);

                    lbpressed = true;
                } else if (!gamepad1.x)
                {
                    lbpressed = false;
                }

            if (gamepad2.x) {
                robot.light.enableLed(true);
            }

            if (gamepad2.y) {
                robot.light.enableLed(false );
            }

            robot.liftmotor.setPower(gamepad1.left_stick_y);

            telemetry.addData("height of lift", robot.liftmotor.getCurrentPosition());







       telemetry.addData("red width: " , robot.redPipeline.getFoundRect().width);
            telemetry.addData("Is Found", robot.redPipeline.isFound());
            telemetry.addData("redTheshold", robot.redPipeline.redTheshold);
       telemetry.update();

        }
        waitForStart();

        liftto.LiftToDistance(1,12,false);


        robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.BLACK, RevBlinkinLedDriver.BlinkinPattern.BLACK);


        //alignment to red post
        double width; // = 200
        elapsedTime.reset();

        sleep(1000);


        double heading = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        System.out.println("ValleyX heading: " + heading);
        while (opModeIsActive() /*&& (width >= 120 || width < 20)*/ && elapsedTime.seconds() <= 3) {
            if (robot.redPipeline.isFound()) {
                width = robot.redPipeline.getFoundRect().width;
                System.out.println(("ValleyX X = " + robot.redPipeline.getXPosition()));

                if (robot.redPipeline.getXPosition() > 350 /*&& heading <= -30*/) {
                    robot.leftBack.setPower(motorspeed);
                    robot.leftFront.setPower(motorspeed);
                    robot.rightBack.setPower(-motorspeed);
                    robot.rightFront.setPower(-motorspeed);
                } else if (robot.redPipeline.getXPosition() < 325 /*&& heading >= -60*/) {
                    robot.leftBack.setPower(-motorspeed);
                    robot.leftFront.setPower(-motorspeed);
                    robot.rightBack.setPower(motorspeed);
                    robot.rightFront.setPower(motorspeed);
                } else {
                    motorspeed = 0;
                    robot.leftBack.setPower(-motorspeed);
                    robot.leftFront.setPower(-motorspeed);
                    robot.rightBack.setPower(motorspeed);
                    robot.rightFront.setPower(motorspeed);
                    break;
                }
                telemetry.addData("Aligned", robot.redPipeline.getAligned());
                telemetry.addData("Get X", robot.redPipeline.getXPosition());
                telemetry.addData("width", robot.redPipeline.getFoundRect().width);
                telemetry.update();
            }

        }

        elapsedTime.reset();
        while (robot.OpMode_.opModeIsActive() &&
                (robot.liftmotor.isBusy()) /*&& (robot_.liftdowntouch.getState() == true )*/
                && (elapsedTime.seconds() < 1));

        elapsedTime.reset();
        sleep(10000);




        //encodermecha.StartAction(0.5,5,5,5,true);
        //robot.StraifLeft(1);
        //robot.allpower(1);
       // headingdrive.gyroDrive(0.2,10,0);
       //headingdrive.gyroTurn(0.8,-90);
        //sleep(1000);
        //headingdrive.gyroTurn(0.8,0);

        //telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
       // telemetry.update();

        //System.out.println("Valley: distance sensor range "+ robot.sensorRange.getDistance(DistanceUnit.INCH));


        //encodermecha.StartAction(0.1,10,10,10,true);
       // headingdrive.gyroDrive(0.5,5,0);
        //Driveto.DriveToDistance(0.1,1,0);




        //liftto.LiftToDistance(0.9,12);
        //sleep(5000);
        //liftto.LiftToDistance(0.2,-4);

        //colorto.drivetocolor(0.2,1, colorto.Red);

        //robot.arm.setPosition(0.5);
        //sleep(30000);


        //liftto.LiftToDistance(0.5,5, true);




    }
}

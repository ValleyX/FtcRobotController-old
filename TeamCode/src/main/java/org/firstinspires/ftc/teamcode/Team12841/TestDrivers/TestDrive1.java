package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware4motors;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name="DriverTest")
public class TestDrive1 extends LinearOpMode

{
  final double TopGoalBack = 0.58;
    //static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //static final int CYCLE_MS = 1000;     // period of each cycle
   // static final double MAX_POS = 0.8;     // Maximum rotational position
   // static final double MIN_POS = 0.2;     // Minimum rotational position

    // Define class members
    //Servo servo;
   // double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
   // boolean rampUp = true;


    public void runOpMode () {
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.

        // Wait for the start button
       // telemetry.addData(">", "Press Start to scan Servo.");
        //telemetry.update();
        waitForStart();

        RobotHardware4motors robot = new RobotHardware4motors(hardwareMap, this);

        double left;
        double right;
        double speed = 0.5;
        boolean dpadrightisfirst= true;
        boolean dpadleftisfirst= true;
        double checkone;
        double checktwo;
        double frontaveragerpm;
        double checkoneback;
        double checktwoback;
        double backaveragerpm;
        int time = 500; //ms

        waitForStart();

        while (opModeIsActive()) {


            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;


            if ((left == 0) && (right == 0)) {
                telemetry.addLine("still");
            } else {
                telemetry.addLine("moving");
            }


            robot.leftDrivefront.setPower(left);
            robot.rightDrivefront.setPower(right);
            robot.leftDriveback.setPower(left);
            robot.rightDriveback.setPower(right);

            if (gamepad1.right_bumper){
                robot.mouth.setPower(1);
                telemetry.addLine("intake on");
            }
            if (gamepad1.left_bumper){
                robot.mouth.setPower(0);
                telemetry.addLine("intake off");
            }

            if (gamepad1.right_stick_button){
                robot.mouth.setPower(-1);
                telemetry.addLine("intake on backward");
            }

            if (gamepad2.right_bumper) {
                robot.shooterfront.setPower(TopGoalBack);
                robot.shooterback.setPower(TopGoalBack);
                telemetry.addLine("shooter on");
            }
            if (gamepad2.left_bumper) {
                robot.shooterfront.setPower(0);
                robot.shooterback.setPower(0);
                telemetry.addLine("shooter off");
            }
            if (gamepad2.dpad_up) {
                robot.ringpusher.setPosition(robot.frontringpusher_POS);//TODO
                telemetry.addLine("dpad up");
            }
            if (gamepad2.dpad_down) {
                robot.ringpusher.setPosition(robot.backringpusher_POS);
                telemetry.addLine("dpad down");
            }
            if (gamepad2.y) {
                robot.Servoarm.setPosition(robot.ARMUP_POS);
                telemetry.addLine("arm up");
            }
            if (gamepad2.b) {
                robot.Servoarm.setPosition(robot.ARMDOWN_POS);
                telemetry.addLine("arm down");
            }
            if (gamepad2.x) {
                robot.Servohand.setPosition(robot.HANDOPEN_POS); //The open for hand
                telemetry.addLine("hand open");
            }
            if (gamepad2.a) {
                robot.Servohand.setPosition(robot.HANDCLOSE_POS); //The close on hand
                telemetry.addLine("hand close");
            }
            if (gamepad2.dpad_left) {
                if (dpadleftisfirst) {
                    speed = speed - 0.05;
                    robot.shooterfront.setPower(speed);
                    robot.shooterback.setPower(speed);
                    dpadleftisfirst = false;
                }
            }
            else {
                dpadleftisfirst = true;
            }

            if (gamepad2.dpad_right) {
                if (dpadrightisfirst) {
                    speed = speed + 0.05;
                    robot.shooterfront.setPower(speed);
                    robot.shooterback.setPower(speed);
                    dpadrightisfirst = false;
                }
            }
            else {
                dpadrightisfirst = true;
            }

            if (gamepad2.left_stick_button) {
                robot.bucket.setPosition(robot.bucketDown_POS);
                telemetry.addLine("bucket down");
            }
            if (gamepad2.right_stick_button){
                robot.bucket.setPosition(robot.bucketUp_POS);
                telemetry.addLine("bucket up");
            }
/*
            //rpm thing
            checkone = robot.shooterfront.getCurrentPosition();

            sleep(time);

            checktwo = robot.shooterfront.getCurrentPosition();

            frontaveragerpm = (checktwo - checkone) / ((double)time / 1000.0);
            telemetry.addData("rpmfront: ", frontaveragerpm);


            checkoneback = robot.shooterback.getCurrentPosition();

            sleep(time);

            checktwoback = robot.shooterback.getCurrentPosition();

            backaveragerpm = (checktwoback - checkoneback) / ((double)time / 1000.0);

            telemetry.addData("rpmback: ", backaveragerpm);
*/
            telemetry.addData("speed = ",speed );
            telemetry.addData("LeftStick = ", left);
            telemetry.addData("RightStick = ", right);
            telemetry.update();


        }
    }
}
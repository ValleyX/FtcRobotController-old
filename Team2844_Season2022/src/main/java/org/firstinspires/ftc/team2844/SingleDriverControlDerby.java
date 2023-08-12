package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.dataflow.qual.TerminatesExecution;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.LiftTicksToDegreesMath;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;

@Disabled
@TeleOp(name = "1Driver")


public class SingleDriverControlDerby extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);
        LiftTicksToDegreesMath liftTicksToDegrees= new LiftTicksToDegreesMath(robot);
        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elbow.setTargetPosition(0);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        double lift = -gamepad1.right_stick_y;

        double wristPos = 0;
        double elbowpos = 0;
        int elbowDegrees = 0;

        boolean pressUp = false;
        boolean pressDown = false;


        //stick control variables
        double leftY;
        double rightY;

        //speed control variables
        double leftY2;
        double rightX;

        //strafe control variables
        double leftLever;
        double rightLever;


        //timer for elbow
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        final int incrementTimeMs = 10;

        //set servo to open
        robot.claw.setPosition(robot.clawOpen);


        waitForStart();



        //test code to see if the recallibration of the arm and the grabber works (this is for teleop)

        //old drive code
        /*while (opModeIsActive()) {
            double lefty = gamepad1.left_stick_y; // try to change to - so that we can use
            double righty = gamepad1.right_stick_y; // the actual robothardware
            double strafeR = gamepad1.right_trigger;
            double strafeL = gamepad1.left_trigger;


            if (gamepad1.right_trigger > 0) {
                robot.leftFront.setPower(-strafeR);
                robot.leftBack.setPower(strafeR);
                robot.rightBack.setPower(-strafeR);
                robot.rightFront.setPower(strafeR);
            } else if (gamepad1.left_trigger > 0.0) {  //is pressed

                robot.leftFront.setPower(strafeL);
                robot.leftBack.setPower(-strafeL);
                robot.rightBack.setPower(strafeL);
                robot.rightFront.setPower(-strafeL);
            } else {

                robot.leftFront.setPower(lefty);
                robot.leftBack.setPower(lefty);
                robot.rightFront.setPower(righty);
                robot.rightBack.setPower(righty);
            }
            */

        //new drive code
        while(opModeIsActive()) {
            //tank drive vars
            //leftY = -gamepad1.left_stick_y;
            //rightY = -gamepad1.right_stick_y;

            leftY2 = -gamepad1.left_stick_y;
            rightX = -gamepad1.right_stick_x;

            leftLever = gamepad1.left_trigger;
            rightLever = gamepad1.right_trigger;

            //left strafe controls
            if(leftLever > 0.5) {
                robot.leftFront.setPower(leftLever);
                robot.leftBack.setPower(-leftLever);
                robot.rightFront.setPower(-leftLever);
                robot.rightBack.setPower(leftLever);
            }
            //right strafe controls
            else if(rightLever > 0.5) {
                robot.leftFront.setPower(-rightLever);
                robot.leftBack.setPower(rightLever);
                robot.rightFront.setPower(rightLever);
                robot.rightBack.setPower(-rightLever);
            }

            //basic movements (Tank Drive)
            /*else {
                lFMotor.setPower(leftY);
                lBMotor.setPower(leftY);
                rFMotor.setPower(rightY);
                rBMotor.setPower(rightY);
            }*/

            //turn right???
            else if (rightX >= 0.3 ) {
                robot.leftFront.setPower(rightX);
                robot.leftBack.setPower(rightX);
                robot.rightFront.setPower(-rightX);
                robot.rightBack.setPower(-rightX);
            }

            //turn left???
            else if (rightX <= -0.3) {
                robot.leftFront.setPower(rightX);
                robot.leftBack.setPower(rightX);
                robot.rightFront.setPower(-rightX);
                robot.rightBack.setPower(-rightX);
            }

            //forwards
            else if (leftY2 >= 0.3) {
                robot.leftFront.setPower(-leftY2);
                robot.leftBack.setPower(-leftY2);
                robot.rightFront.setPower(-leftY2);
                robot.rightBack.setPower(-leftY2);
            }

            //backwards
            else if (leftY2 <= -0.3) {
                robot.leftFront.setPower(-leftY2);
                robot.leftBack.setPower(-leftY2);
                robot.rightFront.setPower(-leftY2);
                robot.rightBack.setPower(-leftY2);
            }

            else {
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);
            }

            telemetry.addData("leftY2", leftY2);
            telemetry.addData("rightX", rightX);
            telemetry.update();



            //elbow stuff
            if (gamepad1.dpad_up) {

                if (elbowDegrees <= 80 && pressUp == false)  {

                    timer.reset();

                    elbowDegrees += 1;

                    // robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setPower(0.5);

                    pressUp = true;

                }
            }
            else if (gamepad1.dpad_down) {

                if (elbowDegrees >= 0 && pressDown == false) {

                    timer.reset();

                    elbowDegrees -= 1;

                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setPower(0.5);

                    pressDown = true;

                }

            }


            //robot.elbow.setPower(0);

            if (timer.milliseconds() > incrementTimeMs) {

                pressUp = false;
                pressDown = false;

            }


            //Still needs to be tested
            //winch setting power if it does not extend to far the current limit is 36 inches
            if (gamepad1.y && robot.winch.getCurrentPosition() < 36)
            {
                robot.winch.setPower(0.9);

            }

            else if (gamepad1.a)
            {
                robot.winch.setPower(-0.9);
            }

            else
            {
                robot.winch.setPower(0);
            }
            //winch

            //turntable
            if (gamepad1.dpad_left)
            {
                robot.turnTable.setPower(0.5);
            }

            else if (gamepad1.dpad_right)
            {
                robot.turnTable.setPower(-0.5);
            }
            else
            {
                robot.turnTable.setPower(0);
            }
            //turntable


            if (gamepad1.right_bumper)
            {
                robot.claw.setPosition(0.56);
            }

            if (gamepad1.left_bumper)
            {
                robot.claw.setPosition(1);
            }

            elbowpos = robot.elbow.getCurrentPosition();
            wristPos = liftMaths.armServoPower(elbowpos);
            robot.wrist.setPosition(wristPos);

            telemetry.addData("wrist positiion", robot.wrist.getPosition());
            telemetry.addData("elbow positiion", robot.elbow.getCurrentPosition());
            telemetry.addData("wrist pos number", wristPos);
            telemetry.addData("elbowDegrees", elbowDegrees);


            telemetry.update();


            //test code to see if the recallibration of the arm and the grabber works (this is for teleop)


            //gyroMove.driveStraight(1, 10, 0);
            //gyroMove.turnToHeading(1, 90);


        /*if (anolgStickup)
        {
            armMotor(up)
            handservoup(armservotopower(armMotortics))
        }
        */
        }
    }
}

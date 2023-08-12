package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.LiftTicksToDegreesMath;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;
@Disabled
@TeleOp(name = "TeleopDriver")


public class TeleopDriver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);
        LiftTicksToDegreesMath liftTicksToDegrees= new LiftTicksToDegreesMath(robot);
        RobotArmDriver_Position armToPosition = new RobotArmDriver_Position(robot);

        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elbow.setTargetPosition(0);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        double winchstick;
        double turntablestick;

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



        waitForStart();

        //new drive code
        while(opModeIsActive()) {


            winchstick =  gamepad2.right_stick_y;
            turntablestick  = gamepad2.left_stick_x;
        //gamepad1 controls
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
        //gamepad1 stuff

        //gamepad2 stuff
            //elbow stuff
            if (gamepad2.dpad_up) {

                if (elbowDegrees <= 80 && pressUp == false)  {

                    timer.reset();

                    elbowDegrees += 1;

                    // robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setPower(0.5);

                    pressUp = true;

                }
            }
            else if (gamepad2.dpad_down) {

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

            if (gamepad2.y)
            {
                robot.winch.setPower(1);
            }
            else if (gamepad2.a)
            {
                robot.winch.setPower(-1);
            }
            else
            {
                robot.winch.setPower(0);
            }

            if (gamepad2.dpad_left)
            {
                robot.turnTable.setPower(0.5);
            }
            else if (gamepad2.dpad_right)
            {
                robot.turnTable.setPower(-0.5);
            }
            else
            {
                robot.turnTable.setPower(0);
            }
           // and robot.winch.getCurrentPosition()

/*
            if (gamepad1.a) {
                armToPosition.setMasterArmPos(0.6, 0, 1, 2, 0.6, 0);
            }

 */


            if (gamepad2.right_bumper)
            {
                robot.claw.setPosition(0.56);
            }

            if (gamepad2.left_bumper)
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

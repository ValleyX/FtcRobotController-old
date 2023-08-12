package org.firstinspires.ftc.team2844;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.dataflow.qual.TerminatesExecution;
import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.LiftTicksToDegreesMath;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;


@TeleOp(name = "OneControllerDrive")

public class EttingerDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);

        robot.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);
        LiftTicksToDegreesMath liftTicksToDegrees= new LiftTicksToDegreesMath(robot);
        RobotArmDriver_Position armToPosition = new RobotArmDriver_Position(robot);




        // robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.elbow.setTargetPosition(0);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        double wristPos = 0;
        double elbowPos = 0;
        int elbowDegrees = 0;
        double winchPos = 0;
        int turnTableDegrees = 0;
        double winchSpeed = .75;
        int turnTableSpeed = 1;


        //eblow booleans
        boolean pressUp = false;
        boolean pressDown = false;
        //winch booleans
        boolean winchUp = false;
        boolean winchDown = false;


        //stick control variables
        double leftY;
        double rightY;

        //speed control variables
        double leftY2;
        double rightX;

        //strafe control variables
        double leftLever;
        double rightLever;

        //stoobs stick control variable
        double winchStick;

        //timer for elbow
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        ElapsedTime fieldTimer = new ElapsedTime();
        fieldTimer.reset();

        final int incrementTimeMs = 10;
        // robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
/*
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
*/
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);



        waitForStart();

        //remove this
        // ElbowPID2 elbowPIDThread = new ElbowPID2(robot);
        //Thread elbowThread = new Thread(elbowPIDThread);
        // elbowThread.start();
        //remove this

        //armToPosition.winchToPosition(0.8,20, true);

        // while (opModeIsActive());

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
            double lift = -gamepad2.right_stick_y;
            winchStick = gamepad2.right_stick_y;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.5;// if does funky, remove the * 0.5, that was added to try and slow turning down

            // Read inverse IMU heading, as the IMU heading is CW positive
            //double botHeading = -robot.imu.getAngularOrientation().firstAngle;
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);



            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFront.setPower(frontLeftPower);
            robot.leftBack.setPower(backLeftPower);
            robot.rightFront.setPower(frontRightPower);
            robot.rightBack.setPower(backRightPower);
            // motorFrontLeft.setPower(frontLeftPower);
            // motorBackLeft.setPower(backLeftPower);
            // motorFrontRight.setPower(frontRightPower);
            // motorBackRight.setPower(backRightPower);

            //elbow stuff
            if (gamepad1.dpad_up) {

                if (elbowDegrees <= 80 && pressUp == false )  {

                    timer.reset();

                    elbowDegrees +=1;
                    armToPosition.elbowposinDeg_ = elbowDegrees;
                    // robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    // elbowPIDThread.runElbow(20,2);
                    robot.elbow.setPower(0.8);

                    // pressUp = true;

                }
            }
            else if (gamepad1.dpad_down){

                if (elbowDegrees >= 0 && pressDown == false) {

                    timer.reset();

                    elbowDegrees -= 1;
                    armToPosition.elbowposinDeg_ = elbowDegrees;
                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setPower(0.5);

                    //  pressDown = true;

                }

            }


            //robot.elbow.setPower(0);

            if (timer.milliseconds() > incrementTimeMs) {

                pressUp = false;
                pressDown = false;


            }

            if (timer.milliseconds() > 10) {

                winchDown = false;
                winchUp = false;
            }

            //winch Up
            /*if (gamepad2.right_stick_y <=0 && winchUp == false)
            {
                winchPos+=winchSpeed;
                armToPosition.winchToPosition(0.8,winchPos,false);
                winchUp = true;
                timer.reset();

            }*/

            //new winch controls (uses buttons)
            //winch in
            if (gamepad1.y && winchUp == false)
            {
                winchPos+=winchSpeed;
                armToPosition.winchToPosition(0.8, winchPos, false);
                winchUp = true;
                timer.reset();
            }

            //winch out
            if (gamepad1.a && winchUp == false)
            {
                winchPos-=winchSpeed;
                armToPosition.winchToPosition(0.8, winchPos, false);
                winchDown = true;
                timer.reset();
            }

            //winch Down
            /*if (gamepad2.right_stick_y>=0 && (winchPos-winchSpeed) >= 0 && winchDown == false)
            {
                winchPos-=winchSpeed;
                armToPosition.winchToPosition(0.8,winchPos,false);
                winchDown = true;
                timer.reset();
            }*/






            //turntable
            if (gamepad1.dpad_left)
            {
                turnTableDegrees += turnTableSpeed;
                armToPosition.turnTableToPosition(1,turnTableDegrees,false);

            }

            if (gamepad1.dpad_right)
            {
                turnTableDegrees -= turnTableSpeed;
                armToPosition.turnTableToPosition(1,turnTableDegrees,false);
            }

            //turntable

            //claw
            if (gamepad1.left_bumper)
            {
                robot.claw.setPosition(robot.clawOpen);
            }

            if (gamepad1.right_bumper)
            {
                robot.claw.setPosition(robot.clawClose);
            }
            //claw
            //goes up and out and turns table at once
            /*if (gamepad2.a)
            {
                // armToPosition.setMasterArmPos(0.8, 45, 1, 20, 0.5, 0);
            }
            //goes back into 0 position
            if (gamepad2.b)
            {
                //   armToPosition.setMasterArmPos(0.5,0,1,5,0.5,0);
            }*/

            /*
            if (gamepad1.a) {
                elbowPIDThread.runElbow(0,2);
                //robot.pidElbow(0);
                //robot.elbow.setTargetPosition(0);//comment theese back out after we finish testing pid loops
               /* robot.winch.setTargetPosition();
                robot.turnTable.setTargetPosition(0);
            }
            if (gamepad1.b) {
                elbowPIDThread.runElbow(25,2);
               // robot.pidElbow(25);
                /*robot.elbow.setTargetPosition(25);
                robot.winch.setTargetPosition();
                robot.turnTable.setTargetPosition(0);
            }
            if (gamepad1.x) {
                elbowPIDThread.runElbow(50,2);
               // robot.pidElbow(50);
                /*robot.elbow.setTargetPosition(50);
                robot.winch.setTargetPosition();
                robot.turnTable.setTargetPosition(0);
            }
            */


            elbowPos = robot.elbow.getCurrentPosition();
            wristPos = liftMaths.armServoPower(elbowPos);
            robot.wrist.setPosition(wristPos);
            // System.out.println("valleyX: wristpos " + wristPos);



            telemetry.addData("wrist positiion", robot.wrist.getPosition());
            telemetry.addData("elbow positiion", robot.elbow.getCurrentPosition());
            telemetry.addData("wrist pos number", wristPos);
            telemetry.addData("elbowDegrees", elbowDegrees);
            telemetry.addData("winge Pos",winchPos);
            telemetry.addData("claw is: ",robot.claw.getPosition());

            // telemetry.addData("right stick ", gamepad2.right_stick_y );

            //public int elbowposinDeg_;
            //public double winchposinIN_;
            //public double turntableposinDeg_;

            telemetry.addData("elbowposinDeg_",armToPosition.elbowposinDeg_);
            telemetry.addData("winchposinIn",armToPosition.winchposinIN_);
            telemetry.addData("turntableposinDeg_",armToPosition.turntableposinDeg_);

            telemetry.addData("right joystick position",gamepad2.right_stick_y);

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


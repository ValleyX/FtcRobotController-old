package org.firstinspires.ftc.team2844;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.LiftTicksToDegreesMath;
import org.firstinspires.ftc.team2844.drivers.RobotArmDriver_Position;
import org.firstinspires.ftc.team2844.drivers.RobotAutoDriveByGyro_Linear;


@TeleOp(name = "StoobsUniversal")

public class StoobsUniversal extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        RobotAutoDriveByGyro_Linear gyroMove = new RobotAutoDriveByGyro_Linear(robot);
        LiftMaths liftMaths = new LiftMaths(robot);
        LiftTicksToDegreesMath liftTicksToDegrees= new LiftTicksToDegreesMath(robot);
        RobotArmDriver_Position armToPosition = new RobotArmDriver_Position(robot);

        robot.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        double wristPos = 0;
        double elbowPos = 0;
        int elbowDegrees = 0;
        double winchPos = 0;
        int turnTableDegrees = 0;
        double winchSpeed = 0.7;
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
        double imuOffset = 0;

        //stoobs stick control variable
        double winchStick;

        //timer for elbow
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        ElapsedTime fieldTimer = new ElapsedTime();
        fieldTimer.reset();

        final int incrementTimeMs = 10;
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();



        //new drive code
        while(opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                double lift = -gamepad2.right_stick_y;
                // Read inverse IMU heading, as the IMU heading is CW positive
                double botHeading = -robot.imu.getAngularOrientation().firstAngle - imuOffset;

                if (gamepad1.a && gamepad1.b)
                {
                    imuOffset = -robot.imu.getAngularOrientation().firstAngle;
                }


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
                robot.rightFront.setPower(frontRightPower);
                robot.leftBack.setPower(backLeftPower);
                robot.rightBack.setPower(backRightPower);

/*
                telemetry.addData("rotX", rotX);
                telemetry.addData("rotY", rotY);
                telemetry.addData("front left power", frontLeftPower);
                telemetry.addData("front right power", frontRightPower);
                telemetry.addData("back left power", backLeftPower);
                telemetry.addData("back right power", backRightPower);
                telemetry.addData("denominator", denominator);
                telemetry.update();
*/




            //elbow stuff
            if (gamepad2.dpad_up) {

                if (elbowDegrees <= 80 && pressUp == false )  {

                    timer.reset();

                    elbowDegrees += 1;
                    armToPosition.elbowposinDeg_ = elbowDegrees;
                    // robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                    robot.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(elbowDegrees));
                   // elbowPIDThread.runElbow(20,2);
                    robot.elbow.setPower(0.8);

                   // pressUp = true;

                }
            }
            else if (gamepad2.dpad_down) {

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
            if (lift > 0 && winchUp == false)
            {
                winchPos+=winchSpeed;
                armToPosition.winchToPosition(0.8,winchPos,false);
                winchUp = true;
                timer.reset();

            }
            //winch Down
            if (lift < 0 && (winchPos-winchSpeed) >= 0 && winchDown == false)
            {
                winchPos-=winchSpeed;
                armToPosition.winchToPosition(0.8,winchPos,false);
                winchDown = true;
                timer.reset();
            }






            //turntable
            if (gamepad2.dpad_left)
            {
                turnTableDegrees += turnTableSpeed;
                armToPosition.turnTableToPosition(0.8,turnTableDegrees,false);

            }

            if (gamepad2.dpad_right)
            {
                turnTableDegrees -= turnTableSpeed;
                armToPosition.turnTableToPosition(0.8,turnTableDegrees,false);
            }

            //turntable

            //claw
            if (gamepad2.left_bumper)
            {
                robot.claw.setPosition(robot.clawOpen);
            }

            if (gamepad2.right_bumper)
            {
                robot.claw.setPosition(robot.clawClose);
            }
            //claw
            //goes up and out and turns table at once
            if (gamepad2.a)
            {
               // armToPosition.setMasterArmPos(0.8, 45, 1, 20, 0.5, 0);
            }
            //goes back into 0 position
            if (gamepad2.b)
            {
             //   armToPosition.setMasterArmPos(0.5,0,1,5,0.5,0);
            }

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

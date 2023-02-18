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
@TeleOp(name = "TeleopDriverTest")


public class TeleopDriverTest extends LinearOpMode {
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

        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //new drive code
        while(opModeIsActive()) {


            winchstick = gamepad2.right_stick_y;
            turntablestick = gamepad2.left_stick_x;
            //gamepad1 controls
            leftY2 = -gamepad1.left_stick_y;
            rightX = -gamepad1.right_stick_x;

            leftLever = gamepad1.left_trigger;
            rightLever = gamepad1.right_trigger;

            robot.elbow.setPower(leftY2);
            telemetry.addData("elbowTicks", robot.elbow.getCurrentPosition());
            telemetry.update();

        }


    }
}

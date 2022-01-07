package org.firstinspires.ftc.team2844.TestCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team2844.Drivers.LiftPositionDriver;
import org.firstinspires.ftc.team2844.Drivers.MechaImuDriver;
import org.firstinspires.ftc.team2844.Drivers.RobotHardware;
import org.firstinspires.ftc.team2844.Drivers.RobotHardwareTest;
import org.firstinspires.ftc.team2844.TestDrivers.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.team2844.TestDrivers.SensorOfDistance;

@TeleOp(name="teleop driver")
public class TestTheMechaWheels extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        //the same name for the motor on the phone
        /*
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor duckySpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        DcMotor superintake = hardwareMap.get(DcMotor.class, "superintake");

        //motors have to be reversed because of the gears or the orientation of the motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


         */

        //RobotHardwareTest robottest = new RobotHardwareTest(hardwareMap, this, 145, 120, RobotHardwareTest.cameraSelection.LEFT);
        RobotHardware robot = new RobotHardware(hardwareMap, this, 0,0, RobotHardware.cameraSelection.LEFT);
        MechaImuDriver headingdrive = new MechaImuDriver(robot);
        //LiftDriverTest liftto = new LiftDriverTest(robot);
        LiftPositionDriver liftto = new LiftPositionDriver(robot);

        // test code
        SensorOfDistance blocksense = new SensorOfDistance();

        double LiftTracker = 0;
        double toplayer = 17;
        double middlelayer = 11;
        double lowlayer = 5;
        double bottom = 0;
        double liftspeed = 0.9;
        double zeroAngle = 0;

        boolean enterendgame = false;
        boolean endgame = false;

        boolean initatedlift = false;

        ElapsedTime timer;
        timer = new ElapsedTime();


        robot.init();

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {


            if  (timer.seconds() >= 90 && timer.seconds() <= 118 && robot.blocksensor.getDistance(DistanceUnit.CM) >= 6) {
                robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED,
                        RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }

            else if (timer.seconds() >= 118 && robot.blocksensor.getDistance(DistanceUnit.CM) >= 6) {
                robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE,
                        RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }

            else if (robot.blocksensor.getDistance(DistanceUnit.CM) < 6) {
                robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.GOLD,
                        RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }

            else {
                robot.setblinkin(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE,
                        RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
            }





            //using different buttons on  controller
            double strafeR = gamepad1.right_trigger;
            double strafeL = gamepad1.left_trigger;
            double lefty = -gamepad1.left_stick_y; // try to change to - so that we can use
            double righty = -gamepad1.right_stick_y; // the actual robothardware
            double lift = -gamepad2.right_stick_y;


            //moving
            if (gamepad1.right_trigger > 0){
                robot.leftFront.setPower(-strafeR);
                robot.leftBack.setPower(strafeR);
                robot.rightBack.setPower(-strafeR);
                robot.rightFront.setPower(strafeR);
            }
            else if (gamepad1.left_trigger > 0.0) {  //is pressed

                robot.leftFront.setPower(strafeL);
                robot.leftBack.setPower(-strafeL);
                robot.rightBack.setPower(strafeL);
                robot.rightFront.setPower(-strafeL);
            }
            else {

                robot.leftFront.setPower(lefty);
                robot.leftBack.setPower(lefty);
                robot.rightFront.setPower(righty);
                robot.rightBack.setPower(righty);
            }


            /** for shared hub aligning
             *
             */
            if (gamepad1.left_bumper){
                zeroAngle = headingdrive.GetAngle();
                System.out.println("valleyx Zero Angle " + zeroAngle);
            }

            if (gamepad1.dpad_up) {
                headingdrive.gyroTurn(1, zeroAngle);
            }

            if (gamepad1.dpad_left){
                headingdrive.gyroTurn(1,zeroAngle- 90);
            }

            if (gamepad1.dpad_right){
                headingdrive.gyroTurn(1,zeroAngle+ 90);
            }

            if (gamepad1.dpad_down){
                headingdrive.gyroTurn(1,zeroAngle+ 180);
            }


            /**
             * lift to
             */
            if ((gamepad2.y ) && (initatedlift == false)) {
                liftto.LiftToPosition(liftspeed,toplayer, false);
                initatedlift = true;
            }

            if ((gamepad2.b ) && (initatedlift == false)) {
                liftto.LiftToPosition(liftspeed,middlelayer, false );
                initatedlift = true;

            }

            if( (gamepad2.a) && (initatedlift == false)) {
                liftto.LiftToPosition(liftspeed,lowlayer, false );
                initatedlift = true;


            }

            if ((gamepad2.x) && (initatedlift == false)) {
                liftto.LiftToPosition(liftspeed,bottom, false );
                initatedlift = true;

            }

            if (initatedlift == true && (!liftto.isliftbusy() || (robot.liftdowntouch.getState() == false))) {
                liftto.stop();
                initatedlift = false;
            }

            /** arm and hand */

            if (gamepad2.left_trigger == 0) {
                robot.arm.setPosition(0);
            }

            else {
                robot.arm.setPosition(.63);
            }


            //robot.grab.setPosition(gamepad2.right_trigger);
            if (gamepad2.right_trigger == 0) {
                robot.grab.setPosition(0);
            }
            else {
                robot.grab.setPosition(0.50);
            }


            /**
             * ducky spinner
             */
            if (gamepad2.dpad_right){
                robot.duckySpinner.setPower(-0.5);
            }

            else if (gamepad2.dpad_left){
                robot.duckySpinner.setPower(0.5);
            }

            else {
                robot.duckySpinner.setPower(0);
            }



            robot.liftmotor.setPower(lift);
/*
             if (lift <= -0.7 ) {
                liftMotor.setPower(-0.7);
            }

            else {
                liftMotor.setPower(lift);
            }

 */

            //robot.superintake.setPower(gamepad2.left_stick_y);
            //robot.intake(gamepad2.left_stick_y);

            if ((robot.blocksensor.getDistance(DistanceUnit.CM) <= 5) && (gamepad2.left_stick_y > 0) ) // flase means pressed
            {
                robot.superintake.setPower(0);
            }

            else if ((robot.blocksensor.getDistance(DistanceUnit.CM) > 5) && (gamepad2.left_stick_y < 0)) // is pressed
            {
                robot.superintake.setPower(gamepad2.left_stick_y);
            }

            else {
                robot.superintake.setPower(gamepad2.left_stick_y);
            }


        /*
            else if (robot.intaketouch.getState() == true && timer.seconds() >= 0 && timer.seconds() <= 90) {
                robot.pattern =  RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
                robot.patternlift = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
                robot.superintake.setPower(gamepad2.left_stick_y);
                robot.blinkinLedDriver.setPattern(robot.patter n);
                robot.blinkinLedDriver2.setPattern(robot.patternlift);

            }

            else if (robot.intaketouch.getState() == true && timer.seconds() >= 90 && timer.seconds() <= 120) {
                robot.pattern =  RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
                robot.patternlift = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
                robot.superintake.setPower(gamepad2.left_stick_y);
                robot.blinkinLedDriver.setPattern(robot.pattern);
                robot.blinkinLedDriver2.setPattern(robot.patternlift);
            }


             */



            //telemetry for phone for driving
            telemetry.addData("leftstick y", lefty);
            telemetry.addData("rightstick y", righty);
            telemetry.addData("strafeR y", strafeR);
            telemetry.addData("strafeL y", strafeL);
            telemetry.addData("ducky speed", gamepad2.dpad_left);
            telemetry.addData("Lift Motor speed: ", lift);
            telemetry.addData("intake : ", gamepad2.left_stick_y );
            telemetry.addData("arm", gamepad2.right_stick_x);
            telemetry.addData("arm position", robot.arm.getPosition());
            telemetry.addData("block distance", robot.blocksensor.getDistance(DistanceUnit.CM));

            telemetry.update();

        }
    }
}


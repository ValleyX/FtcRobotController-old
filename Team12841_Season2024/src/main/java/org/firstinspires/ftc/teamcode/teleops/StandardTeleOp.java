package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import RobotHardwares.RobotHardware;
import RobotHardwares.LiftHardware;


@TeleOp(name = "Standard TeleOp")

public class StandardTeleOp extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;

    double y; // Remember, this is reversed!
    double x; // Counteract imperfect strafing
    double rx;

    double frontLeftActualPower;
    double backLeftActualPower;
    double frontRightActualPower;
    double backRightActualPower;

    double slideSpeed = 1.0;

    boolean robotCentric = false;
    boolean minorMode = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this);
        lifthardware = new LiftHardware(robothardware, this);


        waitForStart();
        if (isStopRequested()) return;
        timer.reset();
        while (opModeIsActive()) {

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
//            rx = 0;
            //Bot heading is in radians not degrees!!!!!!!!!!!!
            double botHeading;
            botHeading = -(double) robothardware.getHeadingRadians(); //change this to getCurrentHeading for field centric, 0 for robot centric

            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(rotY + rotX + rx) / denominator;
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            double backRightPower = -(rotY + rotX - rx) / denominator;

            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.update();

            if(gamepad1.right_trigger > 0.0){
                minorMode = true;
            } else {
                minorMode = false;
            }

            robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);

            lifthardware.powerPivot(gamepad2.right_stick_x);
            lifthardware.powerYSlides(gamepad2.left_stick_y);

//            //opens claw if the left manipulator's bumper is pressed and closes if the right bumper is pressed
//            if(gamepad2.left_bumper){
//                lifthardware.openClaw();
//            } else if (gamepad2.right_bumper){
//                lifthardware.closeClaw();
//            }



            if(gamepad2.y){ //if y is pressed move the slides to the high bucket position
                lifthardware.moveYSlides(20, slideSpeed);
            } else if (gamepad2.x){ //if x is pressed retract both slides at the same time
                lifthardware.movePivot(0, 1.0);
                lifthardware.moveYSlides(0, 1.0);
            }



            //IF THE SENSOR'S GAIN IS CHANGED YOU HAVE TO CHANGE THE VALUES AS WELL (test the color sensor on the samples.)
//            if(robothardware.getHue() >= 200){//if is blue, then sets the lights on the robot to solid blue
//                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//            } else if (120 <= robothardware.getHue() && robothardware.getHue() <= 180){//else if gray, then set the lights to solid grey.
//                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
//            } else if (60 < robothardware.getHue() && robothardware.getHue() <=80){ //else if yellow then set the lights to solid Yellow (yellow is sometimes confused with red)
//                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//            } else if (20 <= robothardware.getHue() && robothardware.getHue() <= 60){// else if red, then change lights to red (sometimes confused with yellow).
//                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            } else { //if it is not looking at a sample then set the lights to the default lights (ocean waves for now) (school wants default to be blue and/or white)
//                if(rainbowMode){
//                    robothardware.lights.sePattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
//                } else {
//                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            //    }
//            }

        }
    }
}

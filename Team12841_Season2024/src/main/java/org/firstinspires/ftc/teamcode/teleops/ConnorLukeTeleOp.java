package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import RobotHardwares.RobotHardware;
import RobotHardwares.LiftHardware;


@TeleOp(name = "Connor and Luke - TeleOp Drive")

public class ConnorLukeTeleOp extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;
    LinearOpMode opMode_;

    double y; // Remember, this is reversed!
    double x; // Counteract imperfect strafing
    double rx;


    double minorPower = 5;

    double extendSpeed = 1.0;
    double pivotSpeed = 1.0;

    //Toggles (there is two variables for each so that if statement for toggle works)
    boolean robotCentric = false; //toggle for robot centric vs field centric
    boolean pressingRobo = false;
    boolean rainbowMode = false; //toggle for rainbow mode for our lights(eventually)
    boolean pressingRainbow = false;
    boolean aPress = false; //preset that goes to max yslides height (~44in)
    boolean pressingA = false;
    boolean bPress = false;
    boolean pressingB = false;
    boolean pressingStart = false;
    boolean overRide = false; //toggle for a manipulator e-stop override in case the encoders get off
    boolean pressingOverride = false;
    boolean pressingBack = false; //part of the toggle for override
    double wristPos;
    int counter = 0;

    //holding buttons
    boolean minorMode = false;
    boolean partyMode = false;
    boolean clawBool = false;
    boolean pressingClaw = false;

    int hangCount = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this);
        lifthardware = new LiftHardware(robothardware, this);
        opMode_ = this;


        robothardware.imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;
        timer.reset();
        while (opModeIsActive()) {


            //if the driver's left trigger is pulled turn off field centric
            if(gamepad1.left_trigger > 0.5){
                if(!pressingRobo){
                    robotCentric = !robotCentric;
                    pressingRobo = true;
                }
            } else {
                pressingRobo = false;
            }

            //if the driver presses y, turn the rainbow lights on (eventually)
            if(gamepad1.y){
                    partyMode = true;
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            } else {
                partyMode = false;
            }


            //if the trigger is pressed or the slides are up(10inches) turn on minor mode(1/5 the speed or 1/2.5 the speed)
            if (gamepad1.right_trigger > 0.1){
                minorMode = true;
                minorPower = 3;
            } else {
                minorMode = false;
            }

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (minorMode) //if minor mode true then go a fifth of the speed(or 1/2.5 if the slides are up but minor mode is not pressed).
            {
                y = y/minorPower;
                x = x/minorPower;
                rx = rx/minorPower;
            }


            //Bot heading is in radians not degrees!!!!!!!!!!!!
            double botHeading;
            if(robotCentric){
                botHeading = 0.0;
            } else {
                botHeading = -(double) robothardware.getHeadingRadians(); //change this to getCurrentHeading for field centric, 0 for robot centric
            }

            //code for field centric (Idk how it works, pretty sure it's magic or makes triangles or something)
            //REMEMBER IT USES RADIANS
            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(rotY + rotX + rx) / denominator;
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            double backRightPower = -(rotY + rotX - rx) / denominator;

            //telemetry for everything
            telemetry.addData("rotX", rotX);
            telemetry.addData("rotY", rotY);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("denominator", denominator);
            telemetry.addData("Minor Mode", minorMode);
            telemetry.addData("Servo Pos.", lifthardware.getClawPos());
            telemetry.addData("Wrist Position", robothardware.clawWrist.getPosition());
            telemetry.addData("Extend Power", robothardware.extendMotor.getPower());
            telemetry.addData("Extend throttle power", gamepad2.left_stick_y);
            telemetry.addData("Extend Pos", robothardware.extendMotor.getCurrentPosition());
            telemetry.addData("Minimum Angle", lifthardware.getMinAngle());
            telemetry.addData("Angle", lifthardware.getAngle());

            telemetry.addData("Y Slides Motor Left Encoder", robothardware.ySlidesMotorLeft.getCurrentPosition());
            telemetry.addData("Y Slides Motor Right Encoder", robothardware.ySlidesMotorRight.getCurrentPosition());
            telemetry.addData("Y Slides Height in Inches", lifthardware.getYSlidesHeight());
            telemetry.addData("Extend Slides Length Encoder", robothardware.extendMotor.getCurrentPosition());
            telemetry.update();

            //set the power for all the motors individually
            robothardware.allPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);

            if(-gamepad2.left_stick_x != 0){
                lifthardware.powerExtendLimitless(-gamepad2.left_stick_x* extendSpeed);
                aPress = false;
                bPress = false;
            } else if(!aPress && !bPress){
                lifthardware.stopExtend();
            }


            if(gamepad1.a){ //if the driver presses a the imu is reset
                robothardware.imu.resetYaw();
            }

            //if the dpad up is pressed make the slides move at normal power
            //and if dpad down is pressed move the slides at baby mode speed
            if(gamepad2.dpad_up){
                extendSpeed = 1.0;
            } else if(gamepad2.dpad_down){
                extendSpeed = 0.5;
            }

            if(gamepad2.dpad_right){
                pivotSpeed = 1.0;
            } else if (gamepad2.dpad_left) {
                pivotSpeed = 0.25;
            }

            if(gamepad2.left_stick_y != 0){
                lifthardware.powerPivot(-gamepad2.left_stick_y*pivotSpeed);
                aPress = false;
                bPress = false;
            } else if(!aPress && !bPress) {
                lifthardware.stopPivot();
            }


            if(counter%2 == 0){
                wristPos = robothardware.clawWrist.getPosition();
                counter = 0;
            }
            counter++;

            if(gamepad2.right_stick_y > 0.1){
                robothardware.clawWrist.setPosition(wristPos - 0.035);
            } else if (gamepad2.right_stick_y < -0.1){
                robothardware.clawWrist.setPosition(wristPos + 0.035);
            }


            if(gamepad2.start){
                if(!pressingStart){
                    hangCount++;
                    pressingStart = true;
                }
            } else {
                pressingStart = false;
            }

            if(hangCount == 1){
                lifthardware.moveYSlides(9.5,1);
                // lifthardware.movePivot(100, 1);
            } else if (hangCount == 2) {
                lifthardware.moveYSlidesAuto(0, 1);
            } else if (hangCount == 3){
                hangCount = 0;
            }

//            if(gamepad2.guide){
//                if(!pressingOverride){
//                    overRide = !overRide;
//                    if(!overRide){
//                        robothardware.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    }
//                    pressingOverride = true;
//                }
//            } else {
//                pressingOverride = false;
//            }

            if(gamepad2.right_bumper){
                if(!pressingClaw){
                    clawBool = !clawBool;
                    pressingClaw = true;
                }
            } else {
                pressingClaw = false;
            }

            if (clawBool){ //if a is pressed move the ySlides down at the power of slideSpeed (baby mode or full power)
                lifthardware.openClaw(); //set the power of the slides (all motor saving is in liftHardware)
            } else {
                lifthardware.closeClaw();
            }

//            if(gamepad2.x){
//                if(!pressingX){
//                    overRide = !overRide;
//                    pressingX = true;
//                }
//            } else {
//                pressingX = false;
//            }

            if(gamepad2.a){
                if(!pressingA){
                    bPress = false;
                    aPress = !aPress;
                    pressingA = true;
                }
            } else {
                pressingA = false;
            }

            if(aPress){
                lifthardware.movePivot(210, pivotSpeed);
                 if(lifthardware.getAngle() >= 120) {
                     robothardware.clawWrist.setPosition(0);
                     lifthardware.moveExtend(20, extendSpeed);
                     if (lifthardware.getExtend() >= 20 && lifthardware.getAngle() >= 210) {
                         aPress = false;
                     }
                 }
            }

            if(gamepad2.b){
                if(!pressingB){
                    aPress = false;
                    bPress = !bPress;
                    pressingB = true;
                }
            } else {
                pressingB = false;
            }

            if(bPress){
                robothardware.clawWrist.setPosition(0.4);
                lifthardware.movePivot(lifthardware.getMinAngle()+17, 1);
                if(lifthardware.getAngle() <= lifthardware.getMinAngle()+17){
                    bPress = false;
                }
            }

            // grn whole match blink ylw 2 at strt endgm 15s blink red game end partymode
            if (!partyMode) {
                if (timer.seconds() > 120)
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
                else if (timer.seconds() > 105) {
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                } else if (timer.seconds() > 92) {
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                } else if (timer.seconds() > 90) {
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                } else {
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
            }


            // separate if statements means that both slides can move at the same time
            // ySlides in has priority over ySlides out



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
//                }
//            }

        }
    }
}

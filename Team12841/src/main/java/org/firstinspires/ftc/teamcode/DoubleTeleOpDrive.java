package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import RobotHardwares.Camera;
import RobotHardwares.LiftHardware;
import RobotHardwares.RobotHardware;

@TeleOp(name = "Two People")
public class DoubleTeleOpDrive extends LinearOpMode {
    RobotHardware robothardware;
    LiftHardware lifthardware;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        robothardware = new RobotHardware(this, Camera.SkystoneDeterminationPipeline.RobotPos.BlueL);
        lifthardware = new LiftHardware(robothardware, this);

        robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double y; // Remember, this is reversed!
        double x; // Counteract imperfect strafing
        double rx;
        int liftCount = 0;
        boolean minorMode = false; //makes the driving slower
        boolean plane = false;
        boolean bucket = false;
        boolean bPress = false;
        boolean down = false;

        //elbow variables
        int elbowPosition;
        boolean elbowStickMovement = false;
        final int STRAIGHT_UP = 3438;
        final int ELBOW1 = 4544;
        final int ELBOW2 = 4509;
        final int ELBOW3 = 4509;
        final int ELBOW4 = 4609;
        final int BOARD_LIFT1 = 0;
        final int BOARD_LIFT2 = 412;
        final int BOARD_LIFT3 = 1049;
        final int BOARD_LIFT4 = 1693;
        final int MAX_ELBOW = 4670;

        //elevator variables
        int elevatorPosition;
        boolean elevatorStickMovement = false;
        final int MAX_ELEVATOR = -4000;

        timer = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;
        timer.reset();

        while (opModeIsActive()) {
            elbowPosition = robothardware.elbowMotor.getCurrentPosition();
            elevatorPosition = robothardware.elevatorMotor.getCurrentPosition();
            /*y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.2;
            rx = gamepad1.right_stick_x;

            //all of the math for field centric
            double botHeading = 0; //change this to getCurrentHeading for field centric, 0 for robot centric
            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;*/

            //if minor mode is on and right bumper is pressed then turn it off
            if (gamepad1.right_bumper) {
                minorMode = !minorMode;
                sleep(200);
            }
            if (minorMode && timer.seconds() < 90)
            {
                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            }else if (!minorMode && timer.seconds() < 90) {
                robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            }
            /*if (minorMode) {
                frontLeftPower /= 10;//divide the speed by 10 to make it very slow
                backLeftPower /= 10;
                frontRightPower /= 10;
                backRightPower /= 10;
            }

            if (gamepad1.dpad_up){
                robothardware.leftFrontDrive.setPower(.1);
                robothardware.leftBackDrive.setPower(.1);
                robothardware.rightFrontDrive.setPower(.1);
                robothardware.rightBackDrive.setPower(.1);
            }
            else if (gamepad1.dpad_down){
                robothardware.leftFrontDrive.setPower(-.1);
                robothardware.leftBackDrive.setPower(-.1);
                robothardware.rightFrontDrive.setPower(-.1);
                robothardware.rightBackDrive.setPower(-.1);
            }
            else if (gamepad1.dpad_right){
                robothardware.leftFrontDrive.setPower(.1);
                robothardware.leftBackDrive.setPower(.1);
                robothardware.rightFrontDrive.setPower(-.1);
                robothardware.rightBackDrive.setPower(-.1);
            }
            else if (gamepad1.dpad_left){
                robothardware.leftFrontDrive.setPower(-.1);
                robothardware.leftBackDrive.setPower(-.1);
                robothardware.rightFrontDrive.setPower(.1);
                robothardware.rightBackDrive.setPower(.1);
            }

            else {
            //makes the motors actually move
                robothardware.leftFrontDrive.setPower(frontLeftPower);
                robothardware.leftBackDrive.setPower(backLeftPower);
                robothardware.rightFrontDrive.setPower(frontRightPower);
                robothardware.rightBackDrive.setPower(backRightPower);
            }*/

            //all the telemetry code
//            telemetry.addData("rotX", rotX);
//            telemetry.addData("rotY", rotY);
            telemetry.addData("elbow ticks", robothardware.elbowMotor.getCurrentPosition());
            telemetry.addData("elevator power", robothardware.elevatorMotor.getPower());
            telemetry.addData("elevator ticks", robothardware.elevatorMotor.getCurrentPosition());
            telemetry.addData("airplane position", robothardware.airplane_launcher.getPosition());
//            telemetry.addData("front left power", frontLeftPower);
//            telemetry.addData("front right power", frontRightPower);
//            telemetry.addData("back left power", backLeftPower);
//            telemetry.addData("back right power", backRightPower);
//            telemetry.addData("denominator", denominator);
            telemetry.update();


            //all of the button commands for the elbow/elevator
            if (gamepad2.y)
                lifthardware.moveElbow(STRAIGHT_UP, 1);
            else if (gamepad2.b|| bPress){
                if(liftCount == 0)
                {
                    lifthardware.moveElbow(ELBOW1, .8);
                }
                //sleep(2000);
                if(robothardware.elbowMotor.isBusy()) {
                    bPress = true;
                }
                else
                {
                    switch(liftCount){
                        case(0):
                            lifthardware.moveElevator(BOARD_LIFT1, 1);
                            liftCount++;
                            sleep(100);
                            break;
                        case(1):
                            lifthardware.moveElevator(BOARD_LIFT2, 1);
                            lifthardware.moveElbow(ELBOW2, 1);
                            liftCount++;
                            sleep(100);
                            break;
                        case(2):
                            lifthardware.moveElevator(BOARD_LIFT3, 1);
                            lifthardware.moveElbow(ELBOW3, 1);
                            liftCount++;
                            sleep(100);
                            break;
                        case(3):
                            lifthardware.moveElevator(BOARD_LIFT4, 1);
                            lifthardware.moveElbow(ELBOW4, 1);
                            liftCount++;
                            sleep(100);
                            break;
                        default:
                            lifthardware.moveElevator(BOARD_LIFT1, 1);
                            liftCount++;
                            sleep(200);
                    }
                    if(liftCount > 3)
                    {
                        liftCount = 0;
                    }
                    bPress = false;
                }
            }
            else if (gamepad2.x || down) {
                if (elbowPosition > -10)
                {
                    bucket = false;
                    lifthardware.closeBucket();
                    liftCount = 0;
                    if(elbowPosition > 3800)
                    {
                        lifthardware.moveElbow(3800, 1);
                    }
                    lifthardware.moveElevator(0, 1);
                    if(robothardware.elevatorMotor.isBusy()) {
                        down = true;
                    }
                    else
                    {
                        lifthardware.moveElbow(0, 1);
                        down = false;
                    }
                }
            } else if (gamepad2.a)
                lifthardware.moveElevator(0, 0.6);

                //the manual stick movement for the elbow
            else if (Math.abs(-gamepad2.left_stick_y) > 0.2) {
                if (elbowPosition >= 0 && elbowPosition <= MAX_ELBOW) {
                    robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (robothardware.elbowMotor.getCurrentPosition() < 3000)
                        robothardware.elbowMotor.setPower(-gamepad2.left_stick_y);
                    else
                        robothardware.elbowMotor.setPower(-.3 * gamepad2.left_stick_y);//makes it move slower when close to the board
                    elbowStickMovement = true;
                }
            }

            //the manual stick movement for the elevator
            else if (Math.abs(gamepad2.right_stick_y) > 0.2) {
                if (elevatorPosition <= 20 && elevatorPosition >= MAX_ELEVATOR) {
                    robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robothardware.elevatorMotor.setPower(gamepad2.right_stick_y);
                    if (elevatorPosition >= -BOARD_LIFT2)
                    {
                        lifthardware.moveElbow(ELBOW2, 1);
                    }
                    if (elevatorPosition >= -10)
                    {
                        liftCount = 0;
                    }
                    elevatorStickMovement = true;
                }
            }

            //safety precautions for the elbow
            else if (elbowStickMovement) {
                if (elbowPosition <= 5) {
                    elbowPosition = 0; //go to 0
                    liftCount = 0;
                } else if (elbowPosition > MAX_ELBOW) {
                    elbowPosition = MAX_ELBOW; //go to MAX_ELBOW
                }
                robothardware.elbowMotor.setTargetPosition(elbowPosition);
                robothardware.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowStickMovement = false;
            }

            //safety precautions for the elevator
            else if (elevatorStickMovement) {
                if (elevatorPosition > 0) {
                    elevatorPosition = 0; //go to 0
                } else if (elevatorPosition < MAX_ELEVATOR) { //elevator pos is negative
                    elevatorPosition = MAX_ELEVATOR; //go to MAX_ELBOW
                }
                robothardware.elevatorMotor.setTargetPosition(elevatorPosition);
                robothardware.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorStickMovement = false;
            }

            //intake and outtake commands
            if (gamepad1.left_trigger > 0.1) {
                robothardware.intakeMotor.setPower(gamepad1.left_trigger);
                if(elbowPosition < 1000)
                {
                    lifthardware.openBucket();
                }
            } else {
                robothardware.intakeMotor.setPower(-gamepad1.right_trigger);
                if (!bucket) {
                    lifthardware.closeBucket();
                }
            }

            if (gamepad2.left_bumper)
                if(!plane)
                {
                    robothardware.airplane_launcher.setPosition(0);
                    plane = true;
                    sleep(200);
                }
                else
                {
                    robothardware.airplane_launcher.setPosition(0.2);
                    plane = false;
                    sleep(200);
                }

            if (gamepad2.right_bumper)
                if(!bucket)
                {
                    lifthardware.openBucket();
                    bucket = true;
                    sleep(200);
                }
                else
                {
                    lifthardware.closeBucket();
                    bucket = false;
                    sleep(200);
                }

            if (Math.abs(gamepad1.left_stick_x) > .01 || Math.abs(gamepad1.left_stick_y) > .01 || Math.abs(gamepad1.right_stick_x) > .01) {

                if (minorMode && Math.abs(gamepad1.left_stick_x) > .5) {
                    robothardware.strafe(-gamepad1.left_stick_x / 8);

                } else if (Math.abs(gamepad1.left_stick_x) > .5) {
                    robothardware.strafe(-gamepad1.left_stick_x);
                }

                if (minorMode && Math.abs(gamepad1.left_stick_y) > .5) {
                    robothardware.allPower(-gamepad1.left_stick_y / 6);
                } else if (Math.abs(gamepad1.left_stick_y) > .5) {
                    robothardware.allPower(-gamepad1.left_stick_y);
                }

                if (minorMode && Math.abs(gamepad1.right_stick_x) > .01) {
                    robothardware.turnBot(gamepad1.right_stick_x / 6);
                } else if (Math.abs(gamepad1.right_stick_x) > .5) {
                    robothardware.turnBot(gamepad1.right_stick_x);
                }
            }
            else {
                robothardware.allPower(0);
            }

            if (timer.seconds() > 90) {
                if (timer.seconds() < 105)
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else if (timer.seconds() < 115)
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                else if (timer.seconds() < 120)
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                else
                    robothardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            }
        }
    }
}

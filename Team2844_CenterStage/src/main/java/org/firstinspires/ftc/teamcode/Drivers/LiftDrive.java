package org.firstinspires.ftc.teamcode.Drivers;


//Made by Benjamin Ettinger 10/25/23
//driver for LIFT, makes it do the thing :) !!!!


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftDrive {

    private RobotHardware robot_; //gets the robot hardware into this class
    DcMotor liftMotorLeft; //makes motor type
    DcMotor liftMotorRight; //makes motor type
    ElapsedTime timeRunning; //creates a timer object that we can use anywhere here, like a double but for time

    //constructor
    public LiftDrive(RobotHardware robot) {

        robot_ = robot; //linking (pointing) to instantiated RobotHardwareClass

        liftMotorLeft = robot_.liftMotorLeft; //gets motor from RobotHardware
        liftMotorRight = robot_.liftMotorRight; //gets motor from RobotHardware
        timeRunning = new ElapsedTime(); //creates new timer object

    }


    //function to set the lift to a certain height
        //liftInches = how far lift is up in inches
        //liftSpeed = how fast the lift runs 0 to 1
        //errorMargin = how much height error is allowable in the function
        //timeOutMS = how long the program will try to do the thing before it gives up and exits
        //wait = if it is going to stop other functions and just do this, or if it isn't going to do that and basically allows multitasking
    public void liftToHeight (double liftInches, double liftSpeed, double errorMargin, double timeOutMS, boolean wait) {

        //gets the rotations per inch, and then sets the distance that we want the lift to go to
        int newLiftTarget = liftMotorLeft.getCurrentPosition() + (int)(liftInches * robot_.LIFT_COUNTS_PER_INCH);

        //tells motors to use the encoder while running
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the target position to the distance we want the lift to go to
        liftMotorLeft.setTargetPosition(newLiftTarget);
        liftMotorRight.setTargetPosition(newLiftTarget);

        //sets the motors to run to position mode
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion
        liftMotorLeft.setPower(Math.abs(liftSpeed));
        liftMotorRight.setPower(Math.abs(liftSpeed));



        //logic to determine if it should wait for the lift to get to the right spot

        //if we want the function to wait, it will stop all other parts of robot until the lift is in the right spot
        if (wait == true) {

            timeRunning.reset();

            while ((liftMotorLeft.isBusy() || liftMotorRight.isBusy())
                    && (liftMotorLeft.getCurrentPosition() > (newLiftTarget + errorMargin)
                    || liftMotorLeft.getCurrentPosition() < (newLiftTarget - errorMargin))
                    && (liftMotorRight.getCurrentPosition() > (newLiftTarget + errorMargin)
                    || liftMotorRight.getCurrentPosition() < (newLiftTarget - errorMargin))
                    || (timeRunning.milliseconds() < timeOutMS))
            {



            } //while loop end bracket

            //commit STOP
            liftMotorLeft.setPower(0);
            liftMotorRight.setPower(0);

        } //if wait==true statement end bracket




    }   //liftToHeight method end bracket



    //steps the lift UP a pixel height (about 3 or so inches I think)
    public void liftStepUp () { //steps the lift up a row

        //check to make sure that we are not at max height
        if (liftMotorLeft.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT || liftMotorRight.getCurrentPosition() >= robot_.MAX_LIFT_HEIGHT) {

            //stop the motors from breaking the robot if we are at the top of our reach
            liftMotorLeft.setPower(0);
            liftMotorRight.setPower(0);

        }
        else { //if we aren't going to break ourselves, go do the thing

            //sets lift target
            int newLiftTarget = liftMotorLeft.getCurrentPosition() + (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);

            //sets the target position to the distance we want the lift to go to
            liftMotorLeft.setTargetPosition(newLiftTarget);
            liftMotorRight.setTargetPosition(newLiftTarget);

            //tells motor to commit do with go to position mode
            liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //start motion
            liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
            liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

        } //else logic statement end bracket

    }   //liftStepUp method bottom bracket


    //steps the lift DOWN a notch essentially
    public void liftStepDown () { //steps the lift up a row

        if (liftMotorLeft.getCurrentPosition() <= robot_.MIN_LIFT_HEIGHT || liftMotorRight.getCurrentPosition() <= robot_.MIN_LIFT_HEIGHT) {

            //stop the motors from breaking the robot if we are at the bottom of our reach
            //0 since that should be the bottom
            liftMotorLeft.setPower(0);
            liftMotorRight.setPower(0);

        }
        else { //if we won't break ourselves, then do the thing

            //sets lift target, subtracts the step value since its going down
            int newLiftTarget = liftMotorLeft.getCurrentPosition() - (int)(robot_.LIFT_STEP * robot_.LIFT_COUNTS_PER_INCH);

            //sets the target position to the distance we want the lift to go to
            liftMotorLeft.setTargetPosition(newLiftTarget);
            liftMotorRight.setTargetPosition(newLiftTarget);

            //tells motor to use run to pos mode when told to run
            liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //start motion, lift speed is the max speed of the lift for accuracy
            liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
            liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

        } //else logic statement end bracket

    }   //liftStepDown method bottom bracket


    //resets lift to min height
    public void liftReset() {

        int newLiftTarget = 0; //sets target to the min height

        //sets the motors to go to the target
        liftMotorLeft.setTargetPosition(newLiftTarget);
        liftMotorRight.setTargetPosition(newLiftTarget);

        //makes motor commit do (go to the position)
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion, lift speed is max speed allowed for the lift so it is speedy but accurate
        liftMotorLeft.setPower(Math.abs(robot_.LIFT_SPEED));
        liftMotorRight.setPower(Math.abs(robot_.LIFT_SPEED));

    } //liftReset method end bracket



}   //class end bracket

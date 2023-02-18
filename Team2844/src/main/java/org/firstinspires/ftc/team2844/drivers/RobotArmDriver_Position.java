package org.firstinspires.ftc.team2844.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.RobotHardware;
import org.firstinspires.ftc.team2844.TeleopDriver;
import org.opencv.core.Mat;

public class RobotArmDriver_Position {


    private RobotHardware robot_;
    private LiftMaths liftMaths;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private LiftTicksToDegreesMath liftTicksToDegrees;
    public int elbowposinDeg_;
    public double winchposinIN_;
    public double turntableposinDeg_;

    double elbowpos = 0;
    double wristPos = 0;

    // Constructor setup all class variables here
    public RobotArmDriver_Position(RobotHardware robot)  {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        elbowposinDeg_ = 0;
        winchposinIN_ = 0;
        turntableposinDeg_ = 0;
        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.elbow.setTargetPosition(0);
        robot.winch.setTargetPosition(0);
        robot.turnTable.setTargetPosition(0);

        //robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftTicksToDegrees= new LiftTicksToDegreesMath(robot);

        liftMaths = new LiftMaths(robot);

    }

    public void winchToPosition (double speed,
                                double positionInInches,
                                boolean waiting) {

        waiting_ = waiting;
        int LiftTarget;
        int moveCounts;
        winchposinIN_ = positionInInches;



        // Ensure that the opmode is still active
        if (robot_.OpMode_.opModeIsActive()) {


            // Determine new target positionInInches, and pass to motor controller
            LiftTarget = (int) (positionInInches * robot_.LIFT_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            // used to determine direction of the front

            robot_.winch.setTargetPosition(LiftTarget);

          //  robot_.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.winch.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() && robot_.winch.isBusy() && (waiting == true)) {// true means not touched {

                robot_.OpMode_.telemetry.addData("lift positionInInches : ", robot_.winch.getCurrentPosition());
                robot_.OpMode_.telemetry.addData("lift target positionInInches : ", robot_.winch.getTargetPosition());
                robot_.OpMode_.telemetry.update();

                //System.out.println("valleyX: " + robot_.winch.getCurrentPosition());
                //System.out.println("valleyX");

            }

            if (waiting == true) {
             //   winchstop();

            }
        }
    }

    public boolean isliftbusy() {
        return (robot_.winch.isBusy());
    }

    public void winchstop()
    {
        robot_.winch.setPower(0);
        waiting_ = false;
    }


    public void elbowToPosition (
                                 double speed,
                                 int positionInDeg,
                                 boolean waiting) {

        waiting_ = waiting;
        int LiftTarget;
        elbowposinDeg_ = positionInDeg;



        if (robot_.OpMode_.opModeIsActive()) {

            robot_.elbow.setTargetPosition(liftTicksToDegrees.liftTicktoDegrees(positionInDeg));

          //  robot_.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.elbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() && robot_.elbow.isBusy() && (waiting == true)) {// true means not touched {

                robot_.OpMode_.telemetry.addData("lift positionInTicks : ", robot_.elbow.getCurrentPosition());
                robot_.OpMode_.telemetry.addData("lift target positionInTicks : ", robot_.elbow.getTargetPosition());

                robot_.OpMode_.telemetry.update();

                //System.out.println("valleyX: elbow ticks " + robot_.elbow.getCurrentPosition());
                //System.out.println("valleyX: elbow target ticks " + robot_.elbow.getTargetPosition());
                //System.out.println("valleyX");
                elbowpos = robot_.elbow.getCurrentPosition();
                wristPos = liftMaths.armServoPower(elbowpos);
                robot_.wrist.setPosition(wristPos);

            }

            if (waiting == true) {
               // elbowstop();

            }
        }
        if (waiting == true) {
            robot_.elbow.setPower(0.8);  //to hold it
            elbowpos = robot_.elbow.getCurrentPosition();
            wristPos = liftMaths.armServoPower(elbowpos);
            robot_.wrist.setPosition(wristPos);
        }
    }

    public void elbowstop()
    {
        robot_.elbow.setPower(0);
        waiting_ = false;
    }

    final int closeTicks = 5;//8

    public boolean turnTableCloseEnough()
    {
        return (Math.abs(turntableposinDeg_ - robot_.turnTable.getCurrentPosition()) > closeTicks);
    }

    public void turnTableToPosition (double speed, //elbowToPosition
                                 double positionInDeg,
                                 boolean waiting) {

        waiting_ = waiting;
        int LiftTarget;
        turntableposinDeg_ = positionInDeg;
        if (robot_.OpMode_.opModeIsActive()) {

            // Determine new target positionInDeg based on degree, and pass to motor controller
            LiftTarget = (int) (positionInDeg * robot_.LIFT_TURNTABLE_COUNTS_PER_DEGREE);

            // Set Target and Turn On RUN_TO_POSITION
            // used to determine direction of the front

            robot_.turnTable.setTargetPosition(LiftTarget);

            robot_.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot_.turnTable.setPower(speed * 1.5);


            // keep looping while we are still active, and BOTH motors are running.
            while (robot_.OpMode_.opModeIsActive() && robot_.turnTable.isBusy() && (waiting == true)) {// true means not touched {

                robot_.OpMode_.telemetry.addData("lift positionInDeg : ", robot_.turnTable.getCurrentPosition());
                robot_.OpMode_.telemetry.addData("lift target positionInDeg : ", robot_.turnTable.getTargetPosition());
                robot_.OpMode_.telemetry.update();

                //System.out.println("valleyX turn table: " + robot_.turnTable.getCurrentPosition());
                //System.out.println("valleyX");

            }

            if (waiting == true) {
               // turntablestop();

            }
        }
    }

    public void turntablestop()
    {
       // robot_.turnTable.setPower(0);
      //  robot_.turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void setClawPos (double position,
                            boolean waiting) {

        robot_.claw.setPosition(position); //range 1 to 0.56


        }

    public void setMasterArmPos (double elbowspeed, int elbowposinDeg,
                                 double winchspeed, double winchposinIN,
                                 double turntablespeed, double turntableposinDeg)
    {
       // elbowposinDeg_ = 0;
       // winchposinIN_ = 0;
       // turntableposinDeg_ = 0;
        if(Math.abs(turntableposinDeg_) > 23 ) {
            elbowToPosition(elbowspeed, 45, true);
            turnTableToPosition(turntablespeed, 0, true);
            winchToPosition(winchspeed,5, true);
            elbowToPosition(elbowspeed, 0, true);
        }
        else
        {
            winchToPosition(winchspeed, winchposinIN, false);
            elbowToPosition(elbowspeed, elbowposinDeg, false);
            turnTableToPosition(turntablespeed, turntableposinDeg, false);
        }

        while (robot_.elbow.isBusy() && robot_.winch.isBusy() && robot_.turnTable.isBusy() && robot_.OpMode_.opModeIsActive())
        {
            robot_.OpMode_.idle();
            elbowpos = robot_.elbow.getCurrentPosition();
            wristPos = liftMaths.armServoPower(elbowpos);
            robot_.wrist.setPosition(wristPos);
        }

        turntablestop();
        elbowpos = robot_.elbow.getCurrentPosition();
        wristPos = liftMaths.armServoPower(elbowpos);
        robot_.wrist.setPosition(wristPos);

       // robot_.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // robot_.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    robot_.turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}

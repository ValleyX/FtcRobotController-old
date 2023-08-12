package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.drivers.LiftMaths;
import org.firstinspires.ftc.team2844.drivers.LiftTicksToDegreesMath;

public class ElbowPID2 implements Runnable
{
    RobotHardware robot_;
     int sensitivity = 2;
   final double Kp = 0.001;  //The portional that changes how agressively it goes after the position
   final double Ki = 0.0002; //The intergral adds speed to the set point and extra power when the point is met
   final double Kd = 0; // The derivative slows down to the set point but can cause instabiablity
    double setPoint = 0;
    double integralSum = 0;
    double lastError = 0;
    int count = 0;
    int ticks = 0;

     double maxSpeed = 0.5;
    ElapsedTime timer;
    LiftMaths liftMaths;


    public ElbowPID2(RobotHardware robot)
    {
        robot_ = robot;
        robot_.elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMaths = new LiftMaths(robot_);
        timer = new ElapsedTime();
    }

    public void runElbow(int setPointDegrees, int tolerance){
        sensitivity = tolerance;
        setPoint = setPointDegrees;
        if (setPoint > robot_.elbow.getCurrentPosition())
            maxSpeed = 0.82; //was 0.82
           // maxSpeed = 1; //was 0.82
        else
            //maxSpeed = 0.27;
            maxSpeed = 0.24;

        integralSum = 0;
        LiftTicksToDegreesMath liftTicksToDegrees;
        liftTicksToDegrees= new LiftTicksToDegreesMath(robot_);
         ticks = (liftTicksToDegrees.liftTicktoDegrees(setPointDegrees));
         setPoint = ticks;
    }
    public boolean isBusy()
    {
        return (Math.abs(ticks+robot_.elbow2.getCurrentPosition() ) >= sensitivity);
    }

    public int targetTicks()
    {
        return ticks;
    }

    public void run()
    {
        while(robot_.OpMode_.opModeIsActive()) {
            double encoderPosition = -robot_.elbow2.getCurrentPosition();
            // calculate the error
            double error = setPoint - encoderPosition;


            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            //System.out.println("valleyX: set point " + setPoint);
            //System.out.println("ValleyX: encoder position " + encoderPosition);

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());
            //limits Intergral to not get intergrater wind up
            if (integralSum > maxSpeed)
                integralSum = maxSpeed;
            if (integralSum < -maxSpeed)
                integralSum = -maxSpeed;
            robot_.OpMode_.telemetry.addData("IntergralSum", integralSum);
            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            //limits output to the top motor speed which is one
            if (out > maxSpeed)
                out = maxSpeed;
            if (out < -maxSpeed)
                out = -maxSpeed;
            robot_.OpMode_.telemetry.addData("out", out);
            //System.out.println("ValleyX: power " + out);
            robot_.OpMode_.telemetry.addData("count", (count++));
            robot_.elbow.setPower(out);
            robot_.OpMode_.telemetry.addData("target pos", ticks);
            robot_.OpMode_.telemetry.addData("currentPos", -robot_.elbow2.getCurrentPosition());
            robot_.OpMode_.telemetry.addData("Power", out);

            // reset the timer for next time
            timer.reset();
            robot_.OpMode_.telemetry.update();
            double  elbowpos = robot_.elbow.getCurrentPosition();  //use elbow not elbow2 for wrist
            double wristPos = liftMaths.armServoPower(elbowpos);
            robot_.wrist.setPosition(wristPos);
            if (isBusy()) {
                robot_.OpMode_.sleep(45); //was 48 milliseconds
            }
            else{
                robot_.OpMode_.sleep(48);
            }
        }
    }

}

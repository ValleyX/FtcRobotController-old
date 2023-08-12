package org.firstinspires.ftc.team2844.drivers;

import org.firstinspires.ftc.team2844.RobotHardware;

public class LiftMaths {
    private RobotHardware robot_;
    public LiftMaths(RobotHardware robot) {
        robot_ = robot;
    }







    //this function will convert motor tics to the power settings on the servo on the end of the arm to make it parallel to the ground
    public double armServoPower(double armMotorTics) {

        return ((robot_).ticsToPower * armMotorTics);

    }





}

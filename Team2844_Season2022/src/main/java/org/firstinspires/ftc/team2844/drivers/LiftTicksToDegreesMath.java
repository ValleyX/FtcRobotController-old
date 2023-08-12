package org.firstinspires.ftc.team2844.drivers;

import org.firstinspires.ftc.team2844.RobotHardware;

public class LiftTicksToDegreesMath {

    private RobotHardware robot_;
    public LiftTicksToDegreesMath(RobotHardware robot) {
        robot_ = robot;
    }



    //this function will convert motor tics to the power settings on the servo on the end of the arm to make it parallel to the ground
    public int liftTicktoDegrees(int degrees) {

        return (int) ((robot_.ticksIn90 / 90) * (double)degrees);

    }



}

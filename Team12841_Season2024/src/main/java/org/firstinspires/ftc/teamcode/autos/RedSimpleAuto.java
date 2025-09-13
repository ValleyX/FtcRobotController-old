package org.firstinspires.ftc.teamcode.autos;
//Muwah ha ha ha ha
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RobotHardwares.RobotHardware;

@Autonomous(name = "Red Simple Auto")
@Disabled
public class RedSimpleAuto extends LinearOpMode {
    // Why is Carter so "good" at coding?
    //ummmm he's not? overrated much?
    //like calm down buddy XD
    //also make sure to sign out of your computer so people like me don't make comments in your code; good idea...
    //oi oi oi baka
    RobotHardware robotHardware;

    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        waitForStart();

        //start of auto
        robotHardware.driveStraight(0.3, 35, 0);//move up to submersible, not 48 inch
        //claw moves down here
        robotHardware.driveStraight(0.3,-10,0);//pull specimen onto rack
        robotHardware.turnToHeading(0.3, 90);//turn right towards observation zone. TURNING BROKEN(SPUN IN CIRCLES)
        robotHardware.driveStraight(0.3,32,90);//move towards blocks that we will push
        robotHardware.turnToHeading(0.3,0);//turns left towards opposing team's zone
        robotHardware.driveStraight(0.3,32,0);//drive towards opposing team's zone
        //carter will code strafing, but otherwise:
        robotHardware.turnToHeading(0.3,90);//turn right away from submersible
        robotHardware.driveStraight(0.3,12,90);//drive away from submersible so the samples are on the right.
        robotHardware.turnToHeading(0.3,180);//turn right so your facing the samples. make this whole thing a tighter turn.
        robotHardware.driveStraight(0.3,57,180);//drive towards observation zone and push sample into it.
    }
}
//How much wood could a woodchuck chuck if a woodchuck could chuck wood
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.testcode.OdometryTestDrive;
import org.firstinspires.ftc.teamcode.testcode.RobotHardwareTestVersion;

//robot must pe put in the same place to relatively same position per game to be consistant
//Position distance from corner near board is about 48 inches
@Autonomous(name="AutoBlueNearBoard")
public class BlueNearBoard extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //Using test hardware for now because test hardware has properties needed for practice
        //ToDo want to move code that works in test hardware to hardware
        RobotHardwareTestVersion robot = new RobotHardwareTestVersion(this,true); //checkBlue is true to only find
        OdometryTestDrive odometryDrive = new OdometryTestDrive(robot);
        RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        while(opModeInInit()){
            //to tell user what values the camera sees
            telemetry.addData("r value", robot.pipeline.avgR);
            telemetry.addData("b value", robot.pipeline.avgLeftB);
            telemetry.addData("r2 value", robot.pipeline.avgLeft2R);
            telemetry.addData("b2 value", robot.pipeline.avgLeft2B);
            telemetry.addData("r3 value", robot.pipeline.avgLeft3R);
            telemetry.addData("b3 value", robot.pipeline.avgLeft3B);

            telemetry.addData("teamProp position", robot.pipeline.position);

            telemetry.update();

            position = robot.pipeline.position;


        }
        position = robot.pipeline.position;
       // System.out.println(position);

        //robot sees the marker is on the left position
        if(position == RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left){
            telemetry.addData("HAHA","OdometryTest drive no work");
            odometryDrive.goToPositionSide(-12,0,0.5,0,1); //Test 1

        }

        //middle
        if(position == RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Middle){
            odometryDrive.goToPositionForward(0,12,0.5,0,1);
            telemetry.addData("HAHA","OdometryTest drive no work 2 ");

        }

        //right
        if(position == RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Right){
            odometryDrive.goToPositionSide(12,0,1,0,0.5);
            telemetry.addData("HAHA","OdometryTest drive no work 3 ");

        }




    }

}

package org.firstinspires.ftc.team22076;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name="TestCamera")
public class TestCamera extends LinearOpMode {

    //this does something...
    @Override
    public void runOpMode() throws InterruptedException {
        //This creates an instance of the RobotHardware class.  This class initializes all the robot Hardware
        RobotHardware Robot = new RobotHardware(this);
        //this is IMU drive
        //IMU_Drive eDrive = new IMU_Drive(Robot);

        while (!opModeIsActive())
        {
            //telm to shiow cam finigs
            telemetry.addData("Marker:" , Robot.pipeline.markerPos);
            telemetry.update();
        }

        waitForStart();


    }
}

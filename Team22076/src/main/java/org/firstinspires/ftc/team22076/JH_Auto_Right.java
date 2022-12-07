package org.firstinspires.ftc.team22076;
// this is new...
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//i thing this is important...
@Autonomous(name="Auto Right")
public class JH_Auto_Right extends LinearOpMode{
    final int HIGH_POLE = 214;
    final int MED_POLE = 214;
    final int LOW_POLE = 131;


    //this does something...
    @Override
    public void runOpMode() throws InterruptedException {
        //This creates an instance of the RobotHardware class.  This class initializes all the robot Hardware
        RobotHardware Robot = new RobotHardware(this);
        int marker_value ;
        //this is IMU drive
        IMU_Drive eDrive = new IMU_Drive(Robot);
        RobotHardware.SkystoneDeterminationPipeline.MarkerPos MarkerPosFinal = RobotHardware.SkystoneDeterminationPipeline.MarkerPos.TWO;

        Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            //getting marker position
            telemetry.addData("Marker:", Robot.pipeline.markerPos);
            telemetry.update();
            MarkerPosFinal = Robot.pipeline.markerPos;
            //MarkerPos MP = MarkerPos.ONE;
        }

        //waitForStart();
        //eDrive.MoveInches(.5,12,12,10,true);
        //basic program to do a simple turn (comment out later)
        //eDrive.driveStraight(.5,53,0,5);
        //eDrive.turnToHeading(.8,-90);
        //eDrive.driveStraight(.5, 23, -90, 5);
        //eDrive.turnToHeading(.8, -180);
        //eDrive.driveStraight(.5, 53, -180, 5);

        //close claw some
        Robot.gripservo.setPosition(0.65);  //.25 open .7 is close
        sleep(1500);
        //drive forward
        eDrive.driveStraight(.5, 35, 0,5);
        eDrive.driveStraight(.5, -5, 0,5);
        eDrive.turnToHeading(.5, 45);
       //sleep(5000);
        //Raise arm
        Robot.swivelmotorRight.setTargetPosition(MED_POLE);
        Robot.swivelmotorLeft.setTargetPosition(MED_POLE);
        Robot.swivelmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.swivelmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.swivelmotorRight.setPower(.8);
        Robot.swivelmotorLeft.setPower(.8);
        while (Robot.swivelmotorRight.isBusy() && Robot.swivelmotorLeft.isBusy() && opModeIsActive())
        {
            idle();
        }
        eDrive.driveStraight(.25, 8, 45,5);
        Robot.swivelmotorRight.setTargetPosition(MED_POLE-25);
        Robot.swivelmotorLeft.setTargetPosition(MED_POLE-25);
        Robot.swivelmotorRight.setPower(.25);
        Robot.swivelmotorLeft.setPower(.25);
        while (Robot.swivelmotorRight.isBusy() && Robot.swivelmotorLeft.isBusy() && opModeIsActive())
        {
            idle();
        }
        Robot.gripservo.setPosition(-0.3);  //.25 open .7 is close
        sleep(500);
        eDrive.driveStraight(.25, -10, 45,5);
        eDrive.turnToHeading(.5, 0);
        Robot.swivelmotorRight.setTargetPosition(0);
        Robot.swivelmotorLeft.setTargetPosition(0);
        Robot.swivelmotorRight.setPower(.25);
        Robot.swivelmotorLeft.setPower(.25);
        while (Robot.swivelmotorRight.isBusy() && Robot.swivelmotorLeft.isBusy() && opModeIsActive())
        {
            idle();
        }
        /*
         *   Web Cam section
         */
        //marker_value
        //Drive to parking spot
     /*   eDrive.driveStraight(.5, 28, 0,5);
      */
        if (MarkerPosFinal == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.ONE ){
            eDrive.turnToHeading(.5, -88);
             eDrive.driveStraight(.5, 27,-88,5);
        }
        else if (MarkerPosFinal == RobotHardware.SkystoneDeterminationPipeline.MarkerPos.THREE){
            //eDrive.turnToHeading(.5, 88);
            eDrive.turnToHeading(.5, -90);
            eDrive.driveStraight(.5, -24,-90,5);
        }
        else {
            //eDrive.driveStraight(.5,1,0,5);
        }

    }

}
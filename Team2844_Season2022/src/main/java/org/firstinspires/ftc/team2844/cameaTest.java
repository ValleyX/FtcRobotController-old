package org.firstinspires.ftc.team2844;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp (name="cameaTest")
public class cameaTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware_ = new RobotHardware(this);



        while(opModeInInit()){
            //telemetry.addData("red value", robotHardware_.pipeline.avgLeftRed);
            //telemetry.addData("blue value", robotHardware_.pipeline.avgLeftBlue);

            telemetry.addData("r value", robotHardware_.pipeline.avgLeftR);
            telemetry.addData("g value", robotHardware_.pipeline.avgLeftG);
            telemetry.addData("b value", robotHardware_.pipeline.avgLeftB);

            telemetry.addData("alpha color", robotHardware_.pipeline.color);

            telemetry.update();
        }
        

    }
}

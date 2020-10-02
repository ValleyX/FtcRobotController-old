/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Team12841.TestDrivers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team12841.Drivers.EncoderDrive;
import org.firstinspires.ftc.teamcode.Team12841.Drivers.RobotHardware;

/**
 * This file is designed to test out the EncoderDrive class
 */

@Autonomous(name="Test: Test Encoder Drive", group="Test")
@Disabled
public class TestEncoderDrive extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();

        RobotHardware robot = new RobotHardware(hardwareMap, this);
        EncoderDrive encoderDrive = new EncoderDrive(robot);

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        System.out.println("ValleyX: Starting...");
        System.out.println("ValleyX: Move forward 12 inches ");
        //going forwards 12 inches
        encoderDrive.StartAction(0.6, 12, 12, 5.0, true);

        sleep(2000); 
        //going backwards 12 inches
        System.out.println("ValleyX: Move backwards 12 inches");
        encoderDrive.StartAction(0.5, -12, -12, 5.0, true);

        sleep(2000);
        //testing no wait functions going forward 12 inches
        runtime.reset();
        System.out.println("ValleyX: Move forward 12 inches");
        encoderDrive.StartAction(0.6, 12, 12, 6.0, false);

        //spin here until encoder is complete
        while (opModeIsActive() && !encoderDrive.IsActionDone() && runtime.seconds() < 5.0)
        {
           idle();
        }

        //Spin above is completed
        encoderDrive.StopAction(); //stop all motors started by StartAction
    }

}





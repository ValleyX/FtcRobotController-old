package org.firstinspires.ftc.team22076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/* This code will use the Left Y and Right X to move the robot drive.  This will allow the robot
to turn and move forward at the same time.
 */

@TeleOp(name="JH_DualStickDrive")
public class DualStickDrive extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {

        //This creates an instance of the RobotHardware class.  This class initializes all the robot Hardware
        RobotHardware Robot = new RobotHardware(this);

        double leftStickY;
        double rightStickX;
        double debounceTol = 0.1;

        //Wait for User to push Start on the Controller
        waitForStart();

        //do loop until stop on controller is pressed
        while (opModeIsActive()) {

            leftStickY = -gamepad1.left_stick_y;
            rightStickX = -gamepad1.right_stick_x;

            //System.out.println("ValleyX stickleft " + leftStickY);

            // moving forward
            if (leftStickY >= debounceTol) {
                //check for left turn, reduce power on left side to turn left
                if (rightStickX >= debounceTol) {
                    Robot.leftFront.setPower(leftStickY - rightStickX); //right stick should be pos when pushed left
                    Robot.leftBack.setPower(leftStickY - rightStickX);  //right stick should be pos when pushed left
                    Robot.rightFront.setPower(Range.clip(leftStickY + rightStickX, -1, 1)); //add power if not full Y
                    Robot.rightBack.setPower(Range.clip(leftStickY + rightStickX, -1, 1)); //add power if not full Y
                }
                //right turn,  reduce power on right side to move right
                else {
                    Robot.leftFront.setPower(Range.clip(leftStickY - rightStickX, -1, 1)); //add power if not full -Y);
                    Robot.leftBack.setPower(Range.clip(leftStickY - rightStickX, -1, 1)); //add power if not full -Y);
                    Robot.rightFront.setPower(leftStickY + rightStickX); //right stick should be neg when pushed right
                    Robot.rightBack.setPower(leftStickY + rightStickX);  //right stick should be neg when pushed right
                }
            }
            // moving backwards
            else if (leftStickY <= -debounceTol) {
                //turing left, reduce power on left side to turn left
                if (rightStickX >= debounceTol) {
                    Robot.leftFront.setPower(leftStickY - rightStickX); //right stick should be pos when pushed left
                    Robot.leftBack.setPower(leftStickY - rightStickX);  //right stick should be pos when pushed left
                    Robot.rightFront.setPower(Range.clip(leftStickY + rightStickX, -1, 1)); //add power if not full Y
                    Robot.rightBack.setPower(Range.clip(leftStickY + rightStickX, -1, 1)); //add power if not full Y
                }
                //turing right, reduce power on right side to move right
                else {
                    Robot.leftFront.setPower(Range.clip(leftStickY - rightStickX, -1, 1)); //add power if not full -Y);
                    Robot.leftBack.setPower(Range.clip(leftStickY - rightStickX, -1, 1)); //add power if not full -Y);
                    Robot.rightFront.setPower(leftStickY + rightStickX); //right stick should be neg when pushed right
                    Robot.rightBack.setPower(leftStickY + rightStickX);  //right stick should be neg when pushed right
                }

            }
            //turing left, right stick should be pos when pushed left
            else if (rightStickX >= debounceTol){
                Robot.leftFront.setPower(rightStickX);
                Robot.leftBack.setPower(rightStickX);
                Robot.rightFront.setPower(-rightStickX);
                Robot.rightBack.setPower(-rightStickX);
            }
            else if (rightStickX >= debounceTol){//this is not right need to fix  JAE 10-19-22
                Robot.leftFront.setPower(-rightStickX);
                Robot.leftBack.setPower(-rightStickX);
                Robot.rightFront.setPower(rightStickX);
                Robot.rightBack.setPower(rightStickX);
            }
            else {
                Robot.leftFront.setPower(0);
                Robot.leftBack.setPower(0);
                Robot.rightFront.setPower(0);
                Robot.rightBack.setPower(0);
            }
        }
    }
}

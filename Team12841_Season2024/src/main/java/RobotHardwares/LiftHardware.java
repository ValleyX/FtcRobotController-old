package RobotHardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LiftHardware {

    RobotHardware robotHardware_;
    ElapsedTime timeRunning;

    LinearOpMode opMode_;



    public final double countsPerMotorRev = 28.0; //how many times the encoder counts in one revolution
    public final double ySlidesGearReduction = 40.0; //the gear ratio of revolutions to movement of slides
    public final double pivotGearReduction = 216.0;
    public final double extendGearReduction = 27.0;

    public final double ySlidesMotorEncoderCountsPerRev = countsPerMotorRev * ySlidesGearReduction; // how many times the motor counts adjusted for the gear reduction.
    public final double pivotEncoderCountsPerRev = countsPerMotorRev * pivotGearReduction;
    public final double extendEncoderCountsPerRev = countsPerMotorRev*extendGearReduction;
    public final double liftDiameterMotor = 1.48; //diameter of the spool to pull the slides
    public final double extendWheelDiameter = 1;
    //the circumference of the spool for the slides
    public final double ySlidesCircumference = liftDiameterMotor * Math.PI;
    public final double extendCircumference = extendWheelDiameter * Math.PI;
    public final double pivotTicsPerDegree = pivotEncoderCountsPerRev/360;


    public final double ySlidesDistancePerRev = ySlidesMotorEncoderCountsPerRev / ySlidesCircumference; //how far the slides move (in "inches") per revolution
    public final double extendDistancePerRev = extendEncoderCountsPerRev / extendCircumference;

    public double floorAngle;
    public double minAngle;

    public final double MAX_ANGLE = 210;
    public final double MAX_YSLIDES_EXTEND = ySlidesDistancePerRev * 44;
    public final double MAX_EXTEND = extendDistancePerRev*18;
    public double maxPivot = MAX_ANGLE;



    public LiftHardware(RobotHardware robotHardware, LinearOpMode opMode) {
        //init and set the motors, robotHardware, opMode and Time to the correct settings.
        robotHardware_ = robotHardware;
        opMode_ = opMode;
        timeRunning = new ElapsedTime();
        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.ySlidesMotorLeft.setTargetPosition(0);
        robotHardware_.ySlidesMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware_.ySlidesMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.ySlidesMotorRight.setTargetPosition(0);
        robotHardware_.ySlidesMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotHardware_.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware_.extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        floorAngle = Math.toDegrees(Math.acos(10.0 / 16 + getExtend()));
        minAngle = pivotTicsPerDegree*floorAngle;
    }

    public void moveYSlides(double pos, double speed) //run to position for teleop with y slides
    {
        int targetPos = (int) (pos*ySlidesDistancePerRev); //find the truncated amount of revolutions the encoder needs to turn to move the right amount of inches

        robotHardware_.ySlidesMotorLeft.setTargetPosition(targetPos); //set the target position
        robotHardware_.ySlidesMotorRight.setTargetPosition(targetPos);


        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set them to the correct code
        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotHardware_.ySlidesMotorLeft.setPower(speed); //go to the targeted position (targetPos) at speed given by user
        robotHardware_.ySlidesMotorRight.setPower(speed);

    }


    public double getYSlidesHeight(){
        return robotHardware_.ySlidesMotorLeft.getCurrentPosition()/ySlidesDistancePerRev;
    }


    public void moveYSlidesAuto(double pos, double speed) //run to position for auto with y slides
    {
        int targetPos;
        targetPos = (int) (pos*ySlidesDistancePerRev); //find the truncated amount of revolutions the encoder needs to turn to move the right amount of inches

        robotHardware_.ySlidesMotorLeft.setTargetPosition(targetPos); //set the target position
        robotHardware_.ySlidesMotorRight.setTargetPosition(targetPos);


        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set them to the correct code
        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do {
            robotHardware_.ySlidesMotorLeft.setPower(speed); //go to the targeted position (targetPos) at speed given by user
            robotHardware_.ySlidesMotorRight.setPower(speed);
        }while(opMode_.opModeIsActive() && (robotHardware_.ySlidesMotorLeft.isBusy() && robotHardware_.ySlidesMotorRight.isBusy()));
    }


    //OVERLOADED THIS IS WITH AN OVERRIDE SWITCH
    public void powerYSlides(double speed, boolean overRide){
        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set both slides to the correct mode
        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //If statement to keep the motors safe
        if((0.3 < getYSlidesHeight() && getYSlidesHeight() <= 44) || overRide){
            //if the slides are between or equal to minimun amd maximum revolutions
            robotHardware_.ySlidesMotorLeft.setPower(speed);
            robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
        }
        else if(getYSlidesHeight() < 0.3){
            //if they are less than 0 (either buggy encoder or the slides are too retracted)
            if(speed > 0){
                robotHardware_.ySlidesMotorLeft.setPower(speed);
                robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
            } else{
                stopYSlides();
            }
        }
        else if(getYSlidesHeight() > 44){
            //if the slides are past the max extend point stop trying to go further
            if(speed < 0){
                robotHardware_.ySlidesMotorLeft.setPower(speed);
                robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
            } else{
                stopYSlides();
            }
        }

    }

    //OVERLOADED THIS IS WITHOUT AN OVERRIDE SWITCH
    public void powerYSlides(double speed){
        robotHardware_.ySlidesMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set both slides to the correct mode
        robotHardware_.ySlidesMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //If statement to keep the motors safe
        if((0.3 < getYSlidesHeight() && getYSlidesHeight() <= 44)){
            //if the slides are between or equal to minimun amd maximum revolutions
            robotHardware_.ySlidesMotorLeft.setPower(speed);
            robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
        }
        else if(getYSlidesHeight() < 0.3){
            //if they are less than 0 (either buggy encoder or the slides are too retracted)
            if(speed > 0){
                robotHardware_.ySlidesMotorLeft.setPower(speed);
                robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
            } else{
                stopYSlides();
            }
        }
        else if(getYSlidesHeight() > 44){
            //if the slides are past the max extend point stop trying to go further
            if(speed < 0){
                robotHardware_.ySlidesMotorLeft.setPower(speed);
                robotHardware_.ySlidesMotorRight.setPower(speed); //set the power of the yslide motors to the joystick
            } else{
                stopYSlides();
            }
        }

    }


    public void stopYSlides(){
        robotHardware_.ySlidesMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware_.ySlidesMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotHardware_.ySlidesMotorLeft.setPower(0);
        robotHardware_.ySlidesMotorRight.setPower(0);
    }

    public double getAngle(){
        return (robotHardware_.pivotMotor.getCurrentPosition()/pivotTicsPerDegree) + 30;
    }

    public double getMinAngle(){
        floorAngle = Math.toDegrees(Math.acos(10.0 / (16 + getExtend())));
        return floorAngle;
    }

    public void movePivot(double angle, double speed) //run to position for teleop with x slides
    {
        int targetPos = (int)(pivotTicsPerDegree*(angle-30));
        robotHardware_.pivotMotor.setTargetPosition(targetPos);
        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.pivotMotor.setPower(speed);
    }

    public void movePivotAuto(double angle, double speed) //run to position for teleop with x slides
    {
        int targetPos = (int) (pivotTicsPerDegree*(angle-30));
        robotHardware_.pivotMotor.setTargetPosition(targetPos);
        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        do {
            robotHardware_.pivotMotor.setPower(speed);
        }while(opMode_.opModeIsActive() && (robotHardware_.pivotMotor.getCurrentPosition() <= targetPos-pivotTicsPerDegree || robotHardware_.pivotMotor.getCurrentPosition() >= targetPos+pivotTicsPerDegree));
    }

    public void powerPivot(double speed){
        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(getAngle() <= getMinAngle()){
            if(speed > 0){
                robotHardware_.pivotMotor.setPower(speed);
            } else {
                stopPivot();
            }
        } else if (getAngle() >= maxPivot){
            if(speed < 0){
                robotHardware_.pivotMotor.setPower(speed);
            } else {
                stopPivot();
            }
        } else {
            robotHardware_.pivotMotor.setPower(speed);
        }
    }


    public void powerPivot(double speed, boolean override){
        robotHardware_.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!override) {
            if (getAngle() <= getMinAngle()) {
                if (speed > 0) {
                    robotHardware_.pivotMotor.setPower(speed);
                } else {
                    stopPivot();
                }
            } else if (getAngle() >= maxPivot) {
                if (speed < 0) {
                    robotHardware_.pivotMotor.setPower(speed);
                } else {
                    stopPivot();
                }
            } else {
                robotHardware_.pivotMotor.setPower(speed);
            }
        } else {
            robotHardware_.pivotMotor.setPower(speed);
        }
    }

    public void powerExtend(double speed){
        robotHardware_.extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(getExtend() <= 0){
            if(speed > 0){
                robotHardware_.extendMotor.setPower(speed);
            } else {
                stopExtend();
            }
        } else if(getExtend() >= MAX_EXTEND){
            if(speed < 0){
                robotHardware_.pivotMotor.setPower(speed);
            } else {
                stopPivot();
            }
        }
    }

    public void powerExtendLimitless(double speed){
        robotHardware_.extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware_.extendMotor.setPower(-speed);

    }

    public void moveExtend(double pos, double speed){
        int targetPos = (int)(extendDistancePerRev*pos);
        robotHardware_.extendMotor.setTargetPosition(targetPos);
        robotHardware_.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware_.extendMotor.setPower(speed);
    }

    public void moveExtendAuto(double pos, double speed){
        int targetPos = (int) (extendDistancePerRev*pos);
        robotHardware_.extendMotor.setTargetPosition(targetPos);
        robotHardware_.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do {
            robotHardware_.extendMotor.setPower(speed);
        }while (opMode_.opModeIsActive() && (robotHardware_.extendMotor.getCurrentPosition() <= targetPos-extendDistancePerRev || robotHardware_.extendMotor.getCurrentPosition() >= targetPos+extendDistancePerRev ));
    }

    public double getExtend(){
        return robotHardware_.extendMotor.getCurrentPosition()/extendDistancePerRev;
    }

    public void stopPivot(){
        robotHardware_.pivotMotor.setPower(0);
    }

    public void stopExtend(){
        robotHardware_.extendMotor.setPower(0);
    }


    public void closeClaw(){robotHardware_.claw.setPosition(0.01);}

    public void openClaw(){robotHardware_.claw.setPosition(0.2);}

    public double getClawPos(){return robotHardware_.claw.getPosition();}


    //public void closeBucket()
    //{
    //    robotHardware_.bucket.setPosition(0.6);
    //} //close

    //public void openBucket()
    //{
    //    robotHardware_.bucket.setPosition(0.7); this is how to use servos!!
    //} //open


}

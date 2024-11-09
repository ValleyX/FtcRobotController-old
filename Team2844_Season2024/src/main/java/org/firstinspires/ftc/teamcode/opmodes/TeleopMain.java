package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.commands.commandGroups.HangStepOne;
import org.firstinspires.ftc.teamcode.commands.commandGroups.HangStepTwo;
import org.firstinspires.ftc.teamcode.commands.hangcommands.HangOut;
import org.firstinspires.ftc.teamcode.commands.hangcommands.HangReset;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftLowChamber;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftScore;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendExtake;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendFreeManip;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendHumanPlayer;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendIn;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendIntake;
import org.firstinspires.ftc.teamcode.commands.subextendcommands.SubExtendOut;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp(name = "Teleop")
public class TeleopMain extends CommandOpMode {
    RobotHardware robot_;
    //declare subsystems
    DriveSubsystem m_drive;
    LiftSubsystem m_lift;
    IntakeSubsystem m_intake;

    //declare commands and command groups
    FieldCentricCommand driveCommand;
    HangStepOne hangStep1;
    HangStepTwo hangStep2;
    LiftReset liftReset;
    LiftHighChamber highChamber;
    LiftLowChamber lowChamber;
    LiftToHumanPlayer humanPlayer;
    LiftScore score;
    SubExtendOut subExtendOut;
    SubExtendIn subExtendIn;
    SubExtendIntake intake;
    SubExtendExtake extake;
    SubExtendFreeManip subExtendFreeManip;
    SubExtendHumanPlayer subExtendHumanPlayer;//aka spit



    //buttons - Named based on the which controller and button is used
    //TODO; determine what button does what
    Button manipLeftBumper;
    Button manipRightBumper;//score
    Button manipA;//hp
    Button manipB;//low chamber
    Button manipy;//high chamber
    Button manipX;//reset
    Button manipDpadUp;//hang s1
    Button manipDpadDown;//hang s2
    Button manipDpadRight;
    Button manipDpadLeft;

    Button driverLeftBumper;//intake
    Button driverRightBumper;//extake
    Button driverA;//spit
    Button driverB;
    Button drivery;
    Button driverX;
    Button driverDpadUp;//subextend out
    Button driverDpadDown;//subextend in
    Button driverDpadRight;
    Button driverDpadLeft;


    //declare controllers
   GamepadEx driverPad;//the driver's controller
   GamepadEx manipPad;// the manipulator's controller

    @Override
    public void initialize() /*throws InterruptedException*/ {
        //Initialize robot - Motor, servos, IMU, Sensors
        robot_ = new RobotHardware(this);

        //Initialize subsystems
        m_drive = new DriveSubsystem(robot_.motorFrontLeft, robot_.motorFrontRight, robot_.motorBackLeft, robot_.motorBackRight, robot_.imu);
        m_lift = new LiftSubsystem(robot_.liftMotor, robot_.hangMotor, robot_.bucketServo);
        //TODO:add in with intake being added in
       // m_intake = new IntakeSubsystem(robot_.subExtendMotor, robot_.intakeMotor, robot_.intakeServo, this, robot_.bucketColorSensor, robot_.belowColorSensor);


        //initialize commands and command groups
        driveCommand = new FieldCentricCommand(m_drive, robot_.OpMode_);
        hangStep1 = new HangStepOne(m_lift, robot_.OpMode_);
        hangStep2 = new HangStepTwo(m_lift, robot_.OpMode_);
        liftReset = new LiftReset(m_lift, robot_.OpMode_);
        highChamber = new LiftHighChamber(m_lift, robot_.OpMode_);
        lowChamber = new LiftLowChamber(m_lift, robot_.OpMode_);
        humanPlayer = new LiftToHumanPlayer(m_lift, robot_.OpMode_);
        score = new LiftScore(m_lift, m_drive, robot_.OpMode_);




        /*TODO:add in with intake being added in
        subExtendOut = new SubExtendOut(m_intake, robot_.OpMode_);
        subExtendIn = new SubExtendIn(m_intake, robot_.OpMode_);
        intake = new SubExtendIntake(m_intake, robot_.OpMode_);
        extake = new SubExtendExtake(m_intake, robot_.OpMode_);
        subExtendHumanPlayer = new SubExtendHumanPlayer(m_intake, robot_.OpMode_);
        subExtendFreeManip = new SubExtendFreeManip(m_intake, robot_.OpMode_);*/

        //initialize controllers
        driverPad = new GamepadEx(gamepad1);
        manipPad = new GamepadEx(gamepad2);

        //initialize buttons

        manipRightBumper = (new GamepadButton(manipPad, GamepadKeys.Button.RIGHT_BUMPER));
        manipA = (new GamepadButton(manipPad, GamepadKeys.Button.A));
        manipB = (new GamepadButton(manipPad, GamepadKeys.Button.B));
        manipX = (new GamepadButton(manipPad, GamepadKeys.Button.X));
        manipy = (new GamepadButton(manipPad, GamepadKeys.Button.Y));
        manipDpadUp = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_UP));
        manipDpadDown = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_DOWN));

        //temp
        manipDpadRight = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_RIGHT));
        manipDpadLeft = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_LEFT));


        driverLeftBumper = (new GamepadButton(driverPad, GamepadKeys.Button.LEFT_BUMPER));
        driverRightBumper = (new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_BUMPER));
        driverDpadUp = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_UP));
        driverDpadDown = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_DOWN));
        driverA = (new GamepadButton(driverPad, GamepadKeys.Button.A));

        //set commands to buttons

        manipRightBumper.whenPressed(score,true);// starts command when the button is pressed

        manipA.whenPressed(humanPlayer,true);
        manipB.whenPressed(lowChamber,true);
        manipX.whenPressed(liftReset,true);
        manipy.whenPressed(highChamber,true);
        manipDpadUp.whenPressed(hangStep1,true);
        manipDpadDown.whenPressed(hangStep2,true);



        /*TODO:add in with intake being added in
        driverLeftBumper.whileHeld(intake,true);
        driverRightBumper.whileHeld(extake,true);
        driverDpadUp.whenPressed(subExtendOut,true);
        driverDpadDown.whenPressed(subExtendIn,true);*/




        //register subsystems
        register(m_drive/*,m_intake*/);

        //set default command to the drive subsystem
        m_drive.setDefaultCommand(driveCommand);
        //TODO:add in with intake being added in
        //m_intake.setDefaultCommand(subExtendFreeManip);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();




        // run the scheduler
        //TODO; add telemetry into here if not already in commands or subsystems
        while (!isStopRequested() && opModeIsActive()) {
            run();

            if(gamepad1.guide){
                robot_.imu.resetYaw();
            }



        }
        reset();
    }

}

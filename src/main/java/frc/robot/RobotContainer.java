package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.lang.model.util.ElementScanner14;

import org.w3c.dom.css.RGBColor;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import frc.robot.GamepadAxisButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ShoulderJogUpCmd;
import frc.robot.commands.ShoulderJogDownCmd;
import frc.robot.commands.ShoulderPositionCmd;
import frc.robot.commands.WristJogDownCmd;
import frc.robot.commands.WristJogUpCmd;
import frc.robot.commands.WristPositionCmd;
import frc.robot.commands.BalanceWaitLevelCmd;
import frc.robot.commands.ExtendJogInCmd;
import frc.robot.commands.ExtendJogOutCmd;
import frc.robot.commands.ExtendPositionCmd;
import frc.robot.commands.IntakeJogCmd;
import frc.robot.commands.WristWaitPositionCmd;
import frc.robot.commands.ExtendWaitPositionCmd;
import frc.robot.commands.ShoulderWaitPositionCmd;

public class RobotContainer 
{
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final ExtendSubsystem extendSubsystem = new ExtendSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    public final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
    public final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

    private boolean isPracticeRobot;

    private Field2d m_field;

    double driveDivider = 1.5;

    GamepadAxisButton m_driverDpadUp;
    GamepadAxisButton m_operatorRightYAxisUp;
    GamepadAxisButton m_operatorRightYAxisDown;
    GamepadAxisButton m_operatorLeftYAxisUp;
    GamepadAxisButton m_operatorLeftYAxisDown;
    GamepadAxisButton m_operatorLeftTrigger;
    GamepadAxisButton m_operatorRightTrigger;
    GamepadAxisButton m_operatorDpadUp;
    GamepadAxisButton m_operatorDpadDown;
    GamepadAxisButton m_operatorDpadLeft;
    GamepadAxisButton m_operatorDpadRight;
    GamepadAxisButton m_driverLT, m_driverRT;

    double m_dCreep=0;

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    public RobotContainer() 
    {
        DigitalInput input;
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        
        input = new DigitalInput(9);
        isPracticeRobot = !input.get();
        input.close();

        configureAutonomousCommands();
    
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> getDriverMoveFwdBack(),
                () -> getDriverMoveLeftRight(),
                () -> getDriverRotate(),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();

        // Create and push Field2d to SmartDashboard.
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        // FIXME MUST NOT BE ENABLED WITH FMS!!!
        // FIXME DISABLE THIS BEFORE COMPETITION!
        //PathPlannerServer.startServer(5811); // 5811 = port number. adjust this according to your needs
    }

    private void setBling( int red, int green, int blue )
    {
        int i;
        for( i=0; i<m_ledBuffer.getLength(); i++ )
        {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private double getDriverMoveFwdBack()
    {
        // Handle creeping forward if the driver is pressing D-pad up
        double pos;
        if( m_dCreep != 0 )
            // Use a fixed value to creep forward
            pos = m_dCreep;
        else
            // Use the joystick axis
            pos = driverJoystick.getRawAxis(OIConstants.kDriverYAxis) / driveDivider;
        return pos;
    }

    private double getDriverMoveLeftRight()
    {
        double pos;
        if( m_dCreep != 0 )
            pos = 0;
        else
            pos = driverJoystick.getRawAxis(OIConstants.kDriverXAxis) / driveDivider;
        return pos;
    }

    private double getDriverRotate()
    {
        double pos;
        if( m_dCreep != 0 )
            pos = 0;
        else
            pos = driverJoystick.getRawAxis(OIConstants.kDriverRotAxis) / driveDivider;
        return pos;
    }

    private void DriveSlowDividerSet( double divider )
    {
        driveDivider = divider;
    }
    
    public void setCreep( double value )
    {
        m_dCreep = value;

        if( DriverStation.getAlliance() == DriverStation.Alliance.Blue ){
            m_dCreep = m_dCreep * 1.10;
            //juice blue side a little higher
        }
           
        System.out.println("setCreep " + m_dCreep);
    }

    private void configureButtonBindings() 
    {
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).whenPressed( () -> swerveSubsystem.zeroHeading(0.0) );

        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value)
           .whenPressed(() -> DriveSlowDividerSet(1.0))
           .whenReleased(() -> DriveSlowDividerSet(1.5));
        
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value)
           .whenPressed(() -> DriveSlowDividerSet(2.6))
           .whenReleased(() -> DriveSlowDividerSet(1.5));

        new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value)
           .whenPressed(() -> DriveSlowDividerSet(2.6))
           .whenReleased(() -> DriveSlowDividerSet(1.5));
        
        new JoystickButton(operatorJoystick, XboxController.Button.kY.value)
            .whileTrue( ScoreHighProCmd()
            .finallyDo( this::RumbleConfirm )
            );
        new JoystickButton(operatorJoystick, XboxController.Button.kB.value)
            .whileTrue( ScoreMidProCmd() 
            .finallyDo( this::RumbleConfirm )
            );
        new JoystickButton(operatorJoystick, XboxController.Button.kA.value)
            .whileTrue( ScoreLowCmd() 
            .finallyDo( this::RumbleConfirm )
            );

        new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value)
            .whileTrue( new ConditionalCommand( ShelfLoadConeCmd(), ShelfLoadCubeCmd(), intakeSubsystem::getCone )
            .finallyDo( this::RumbleConfirm )
            );

        new JoystickButton(operatorJoystick, XboxController.Button.kBack.value)
            .whileTrue( 
                new SequentialCommandGroup(
                    new InstantCommand( ()-> shoulderSubsystem.latchStartingPosition() ),
                    new ConditionalCommand( StowCmdLow(), StowCmdHigh(), shoulderSubsystem::startedBelowLevel )
                )
            .finallyDo( this::RumbleConfirm )
            );

        m_driverDpadUp = new GamepadAxisButton(this::driverDpadUp);
        m_driverDpadUp
            .onTrue( new SequentialCommandGroup(
                new InstantCommand( ()-> swerveSubsystem.setRampRate(0.25)),
                new InstantCommand( ()-> setCreep(DriveConstants.CreepLoading) )
                //new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) )
            ) )
            .onFalse( new SequentialCommandGroup(
                new InstantCommand ( ()-> swerveSubsystem.setRampRate(0)),
                new InstantCommand( ()-> setCreep(0) ) 
            ));
    

        new JoystickButton(operatorJoystick, XboxController.Button.kStart.value).whenPressed(() -> extendSubsystem.zeroPosition());

        m_operatorRightYAxisUp = new GamepadAxisButton(this::operatorRightYAxisUp);
        m_operatorRightYAxisUp.whileTrue( new ShoulderJogUpCmd( shoulderSubsystem ) );
        m_operatorRightYAxisDown = new GamepadAxisButton(this::operatorRightYAxisDown);
        m_operatorRightYAxisDown.whileTrue( new ShoulderJogDownCmd( shoulderSubsystem ) );

        m_operatorDpadUp = new GamepadAxisButton(this::operatorDpadUp);
        m_operatorDpadUp.whileTrue( new WristJogUpCmd( wristSubsystem ) );
        m_operatorDpadDown = new GamepadAxisButton(this::operatorDpadDown);
        m_operatorDpadDown.whileTrue( new WristJogDownCmd( wristSubsystem ) );

        m_operatorDpadLeft = new GamepadAxisButton(this::operatorDpadLeft);
        m_operatorDpadLeft.onTrue(
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(true) ),
                new InstantCommand( ()->setBling( 255,120, 0 ) ),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kLeftRumble, 1.0) ),
                new WaitCommand(0.5),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kLeftRumble, 0.0) )
            )
         );
        m_operatorDpadRight = new GamepadAxisButton(this::operatorDpadRight);
        m_operatorDpadRight.onTrue( 
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
                new InstantCommand( ()->setBling( 255,0, 255 ) ),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kRightRumble, 1.0) ),
                new WaitCommand(0.5),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kRightRumble, 0.0) )
            )
        );

        m_operatorLeftYAxisUp = new GamepadAxisButton(this::operatorLeftYAxisUp);
        m_operatorLeftYAxisUp.whileTrue( new ExtendJogOutCmd( extendSubsystem ) );
        m_operatorLeftYAxisDown = new GamepadAxisButton(this::operatorLeftYAxisDown);
        m_operatorLeftYAxisDown.whileTrue( new ExtendJogInCmd( extendSubsystem ) );

        m_operatorLeftTrigger = new GamepadAxisButton(this::operatorLeftTrigger);
        m_operatorLeftTrigger
            .whileTrue( new ConditionalCommand( FloorLoadConeCmd(), FloorLoadCubeCmd(), intakeSubsystem::getCone )
            .finallyDo( this::RumbleConfirm )
        );

        m_driverLT = new GamepadAxisButton(this::DriverLTrigger);
        m_driverLT.whileTrue( new IntakeJogCmd( intakeSubsystem, true ) );
        m_driverRT = new GamepadAxisButton(this::DriverRTtrigger);
        m_driverRT.whileTrue( new IntakeJogCmd( intakeSubsystem, false ) );
    }

    public boolean driverDpadUp()
    {
        return ( driverJoystick.getPOV() == 0 );
    }

    public boolean operatorRightYAxisUp()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightY.value ) < -0.3 );
    }

    public boolean operatorRightYAxisDown()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightY.value ) > 0.3 );
    }

    public boolean DriverLTrigger()
    {
        return ( driverJoystick.getRawAxis( XboxController.Axis.kLeftTrigger.value ) > 0.3 );
    }

    public boolean DriverRTtrigger()
    {
        return ( driverJoystick.getRawAxis( XboxController.Axis.kRightTrigger.value ) > 0.3 );
    }

    
    public boolean operatorLeftYAxisUp()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftY.value ) < -0.3 );
    }

    public boolean operatorLeftYAxisDown()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftY.value ) > 0.3 );
    }

    public boolean operatorLeftTrigger()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftTrigger.value ) > 0.3 );
    }

    public boolean operatorRightTrigger()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightTrigger.value ) > 0.3 );
    }

    public boolean operatorDpadUp()
    {
        return ( operatorJoystick.getPOV() == 0 );
    }

    public boolean operatorDpadDown()
    {
        return ( operatorJoystick.getPOV() == 180 );
    }

    public boolean operatorDpadLeft()
    {
        return ( operatorJoystick.getPOV() == 270 );
    }

    public boolean operatorDpadRight()
    {
        return ( operatorJoystick.getPOV() == 90 );
    }


    public Command StowCmdLow()
    {
        // STOW when the arm starts below level
        return new SequentialCommandGroup(
            // Move SHOULDER up to clear the bumper
            new InstantCommand( ()->System.out.println("StowCmdLow") ),
            new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            new ParallelCommandGroup(
                // Move WRIST all the way in
                new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_STOWED, true),
                new SequentialCommandGroup(
                    // Once WRIST is almost all the way in
                    new WristWaitPositionCmd( wristSubsystem, true, WristConstants.WRIST_POSITION_STOWED - 300 ),
                    // Pull EXTEND all the way in
                    new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
                    // Pull SHOULDER all the way down
                    new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_STOWED, true),
                    new InstantCommand( ()-> wristSubsystem.setPosition(WristConstants.WRIST_POSITION_STOWED) ),
                    new WaitCommand(0.5)
                )
            ),
            new InstantCommand(() -> shoulderSubsystem.setShoulder(0)),
            new InstantCommand(() -> extendSubsystem.setExtend(0)),
            new InstantCommand(() -> intakeSubsystem.setIntake(0)),
            new InstantCommand(() -> wristSubsystem.setWrist(0))
        );
    }

    public Command StowCmdHigh()
    {
        // STOW when the arm is above level
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("StowCmdHigh") ),
            new ParallelCommandGroup(
                // Move EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, false),
                new SequentialCommandGroup(
                    // Wait for EXTEND to be past the shelf
                    new ExtendWaitPositionCmd(extendSubsystem, false, ExtendConstants.EXTEND_POSITION_HIGH_PRO - 3000),
                    // Once EXTEND is in past the shelf, move WRIST all the way in
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_STOWED, true)
                )
            ),
            // Move SHOULDER to STOWED
            new SequentialCommandGroup(
                new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_STOWED, true),
                new WaitCommand(0.5)
            ),
            new InstantCommand(() -> shoulderSubsystem.setShoulder(0)),
            new InstantCommand(() -> extendSubsystem.setExtend(0)),
            new InstantCommand(() -> intakeSubsystem.setIntake(0)),
            new InstantCommand(() -> wristSubsystem.setWrist(0))
        );
    }

    public Command ScoreHighCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ScoreHighCmd") ),
            new ParallelCommandGroup(
                // Pull game piece in a little IF IT IS A CONE
                new ConditionalCommand(
                    new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                    new WaitCommand(0),
                    intakeSubsystem::getCone ),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to high position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_HIGH, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to HIGH position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_HIGH, false)
                )
            ),
            // Wait until wrist is past straight
            new WristWaitPositionCmd(wristSubsystem, false, WristConstants.WRIST_POSITION_STRAIGHT),
            // Move EXTEND to HIGH position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_HIGH, true)
        );
    }

    public Command ScoreHighProCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ScoreHighProCmd") ),
            new ParallelCommandGroup(
                // Pull game piece in a little IF IT IS A CONE
                new ConditionalCommand(
                    new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                    new WaitCommand(0),
                    intakeSubsystem::getCone ),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to high position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_HIGH_PRO, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to HIGH position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_HIGH_PRO, false)
                )
            ),
            // Wait until wrist is past straight
            new WristWaitPositionCmd(wristSubsystem, false, WristConstants.WRIST_POSITION_STRAIGHT),
            // Move EXTEND to HIGH position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_HIGH_PRO, true)
        );
    }

    public Command ScoreMidCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ScoreMidCmd") ),
            new ParallelCommandGroup(
                // Pull game piece in a little IF IT IS A CONE
                new ConditionalCommand(
                    new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                    new WaitCommand(0),
                    intakeSubsystem::getCone ),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to mid position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_MID, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to MID position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_MID, false)
                )
            ),
            // Wait until wrist is past straight
            new WristWaitPositionCmd(wristSubsystem, false, WristConstants.WRIST_POSITION_STRAIGHT),
            // Move EXTEND to MID position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_MID, true)
        );
    }

    public Command ScoreMidProCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ScoreMidProCmd") ),
            new ParallelCommandGroup(
                // Pull game piece in a little IF IT IS A CONE
                new ConditionalCommand(
                    new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                    new WaitCommand(0),
                    intakeSubsystem::getCone ),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to mid position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_MID_PRO, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to MID position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_MID_PRO, false)
                )
            ),
            // Wait until wrist is past straight
            new WristWaitPositionCmd(wristSubsystem, false, WristConstants.WRIST_POSITION_STRAIGHT),
            // Move EXTEND to MID position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_MID, true)
        );
    }

    public Command ScoreLowCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ScoreLowCmd") ),
            new ParallelCommandGroup(
                // Pull game piece in a little IF IT IS A CONE
                new ConditionalCommand(
                    new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                    new WaitCommand(0),
                    intakeSubsystem::getCone ),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to LEVEL position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_LEVEL, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to LOW position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_LOW, true)
                )
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to LOW
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_LOW, true),
                // Move EXTEND to SCORE LOW
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_SCORE_LOW, true)
            )
        );
    }

    public Command ShelfLoadConeCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ShelfLoadConeCmd") ),
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Start moving SHOULDER to SHELF
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF_CONE, false),
            new SequentialCommandGroup(
                // Wait for SHOULDER to be above bumper
                new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                // Move WRIST to SHELF
                new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_SHELF_CONE, true),
                // Wait for SHOULDER to be correct
                new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF_CONE, true),
                // Move EXTEND to SHELF
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_SHELF, true)
            )   
        );
    }

    public Command ShelfLoadCubeCmd(){
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("ShelfLoadCubeCmd") ),
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Move SHOULDER to SHELF
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF_CUBE, false),
            new SequentialCommandGroup(
                // Wait for SHOULDER to be above bumper
                new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                // Move WRIST to SHELF
                new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_SHELF_CUBE, true),
                // Wait for SHOULDER to be correct
                new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF_CUBE, true),
                // Move EXTEND to SHELF
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_SHELF, true)
            )
        );
    }

    public Command FloorLoadConeFromHighCmd()
    {
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("FloorLoadConeFromHighCmd") ),
            // Move WRIST to CONE PICKUP
            new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_CONE_PICKUP, true),
            new ParallelCommandGroup (
                new InstantCommand( ()->wristSubsystem.setWrist(0) ),         
                // Move SHOULDER to CONE PICKUP
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_CONE_PICKUP, true)
            )
        );
    }

    public Command FloorLoadConeCmd()
    {
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("FloorLoadConeCmd") ),
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Move SHOULDER to above bumper
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            // Move WRIST to CONE PICKUP
            new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_CONE_PICKUP, true),
            new ParallelCommandGroup (
                // Move EXTEND to CONE PICKUP
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_CONE_PICKUP, true),
                new InstantCommand( ()->wristSubsystem.setWrist(0) ),         
                // Move SHOULDER to CONE PICKUP
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_CONE_PICKUP, true)
            )
        );
    }

    public Command FloorLoadCubeCmd()
    {
        return new SequentialCommandGroup(
            new InstantCommand( ()->System.out.println("FloorLoadCubeCmd") ),
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Move SHOULDER to above bumper
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            // Move WRIST to CUBE PICKUP
            new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_CUBE_PICKUP, true),
            // Move EXTEND to CUBE PICKUP
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_CUBE_PICKUP, true),
            new InstantCommand( ()->wristSubsystem.setWrist(0) ),         
            // Move SHOULDER to CUBE PICKUP
            new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_CUBE_PICKUP, true)
        );
    }

    public Command RumbleCmd( double rumbleAmount )
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> operatorJoystick.setRumble(RumbleType.kLeftRumble, rumbleAmount) ),
            new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kLeftRumble, rumbleAmount) )
        );
    }
    
    public void RumbleConfirm( boolean interrupted )
    {
        if( interrupted == false )
        {
            // Make a solid rumble to tell the
            // operator+driver that the motion completed
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    RumbleCmd( 1.0 ),
                    new WaitCommand(0.5),
                    RumbleCmd( 0.0 )
                )
            );
        }
        else
        {
            // Make a half-hearted 3 rumble bursts to tell the
            // operator+driver the motion didn't complete
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    RumbleCmd( 0.5 ),
                    new WaitCommand(0.1),
                    RumbleCmd( 0.0 ),
                    new WaitCommand(0.1),
                    RumbleCmd( 0.5 ),
                    new WaitCommand(0.1),
                    RumbleCmd( 0.0 ),
                    new WaitCommand(0.1),
                    RumbleCmd( 0.5 ),
                    new WaitCommand(0.1),
                    RumbleCmd( 0.0 )
                )
            );
        }
    }


    void configureAutonomousCommands()
    {
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Auto Nothing", new WaitCommand(15.0) );
        m_chooser.addOption("Balance", AutoBalanceCmd() );
        m_chooser.addOption("Balance + Mobility", AutoBalanceMobilityCmd());
        m_chooser.addOption("Inner", AutoInnerCmd());
        m_chooser.addOption("Outer", AutoOuterCmd());
        m_chooser.addOption("Mobility", AutoMobilityCmd());

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    public Command getAutonomousCommand() 
    {
        // return the selected Auton
        // called by Robot.java / autonomousInit()
        return m_chooser.getSelected();
    }

    private Command AutoBalanceCmd()
    {
        // Load the PathPlanner path file and generate it with a max
        // velocity and acceleration for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Balance", 
            new PathConstraints(0.55, 2));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command autoBuilderCommand = autoBuilder.fullAuto(pathGroup);

        return new SequentialCommandGroup(
            new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) ),
            new InstantCommand( () -> swerveSubsystem.initialPitch() ),
            new InstantCommand( ()-> intakeSubsystem.setCone(true) ),
            ScoreHighCmd(),
            new WaitCommand(0.5),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(0.5),
            new ParallelCommandGroup(
                StowCmdHigh(),
                autoBuilderCommand
            ),
            new InstantCommand( ()->setCreep(DriveConstants.CreepBalance * 0.9) ),
            new BalanceWaitLevelCmd(swerveSubsystem, 4.0)
                .deadlineWith(
                    new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
                    )
                ),
            new InstantCommand( ()->setCreep(-DriveConstants.CreepBalance * 0.6) ),
            new WaitCommand(0.15)
            .deadlineWith(
                new SwerveJoystickCmd(
                    swerveSubsystem,
                    () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
                )
            ),
            new InstantCommand( ()->setCreep(0) )
        );
    }

    private Command AutoBalanceMobilityCmd()
    {
        // Load the PathPlanner path file and generate it with a max
        // velocity and acceleration for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Balance + Mobility", 
            new PathConstraints(0.9, 2.0));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Stow", StowCmdHigh());
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command autoBuilderCommand = autoBuilder.fullAuto(pathGroup);

        return new SequentialCommandGroup(
            new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) ),
            new InstantCommand( () -> swerveSubsystem.initialPitch() ),
            new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
            ScoreHighProCmd().withTimeout(3.5),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(0.3),
            autoBuilderCommand,
            // After pathplanner mobility finishes halfway up the ramp, creep forward
            new InstantCommand( ()->setCreep(DriveConstants.CreepBalanceMobility) ),
            // Wait for the platform to start tilting
            new BalanceWaitLevelCmd(swerveSubsystem, 9.0)
                .deadlineWith(
                    new SwerveJoystickCmd( swerveSubsystem, () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) )
                ),
            // Once platform starts tilting, stop driving forward
            new InstantCommand( ()->setCreep(0) ),
            new WaitCommand(0.1)
                .deadlineWith(
                    new SwerveJoystickCmd( swerveSubsystem, () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) )
                ),
            // Once platform starts tilting, wait a little for it to settle
            new WaitCommand(1.0),
            // Now drive backwards slowly until the platform is balanced
            new InstantCommand( ()->setCreep(DriveConstants.CreepBalanceMobilityBackup) ),
            // Wait for the platform to be balanced
            new BalanceWaitLevelCmd(swerveSubsystem, 1.0)
                .deadlineWith(
                    new SwerveJoystickCmd( swerveSubsystem, () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) )
                ),
            // Balanced - stop creeping
            new InstantCommand( ()->setCreep(0) ),
            new WaitCommand(0.15)
            .deadlineWith(
                new SwerveJoystickCmd( swerveSubsystem, () -> getDriverMoveFwdBack(), () -> getDriverMoveLeftRight(), () -> getDriverRotate(), () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) )
            )
    );
    }

    private Command AutoInnerCmd()
    {
        // Load the PathPlanner path file and generate it with a max
        // velocity and acceleration for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Inner", 
            new PathConstraints(3.5, 1.3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("FloorLoad",
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
                FloorLoadCubeCmd()
            )
        );

        eventMap.put("LoadLow", 
            new SequentialCommandGroup(
                new IntakeJogCmd( intakeSubsystem, true ).withTimeout(2.0),
                StowCmdLow()
            )
        );

//        eventMap.put("ScoreHigh2",
//            new SequentialCommandGroup(
//                new IntakeJogCmd( intakeSubsystem, false ).withTimeout(1.5),
//                StowCmdHigh()
//            )
//        );

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command autoBuilderCommand = autoBuilder.fullAuto(pathGroup);

        return new SequentialCommandGroup(
            new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) ),
            new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
            ScoreHighProCmd().withTimeout(3.5),
            new WaitCommand(0.5),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(0.5),
            new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
            // Move EXTEND to CUBE PICKUP
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_CUBE_PICKUP, true),
            autoBuilderCommand,
            ScoreMidProCmd(),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(3.0)
        );
    }

    private Command AutoOuterCmd()
    {
        // Load the PathPlanner path file and generate it with a max
        // velocity and acceleration for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Outer", 
            new PathConstraints(3.5, 1.3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("FloorLoadSetup", 
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
                FloorLoadCubeCmd()
            )
        );

        eventMap.put("LoadLow", 
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
                new IntakeJogCmd( intakeSubsystem, true ).withTimeout(2.0)
            )
        );

        /*eventMap.put("ScoreHigh2",
            new SequentialCommandGroup(
                ScoreMidProCmd(),
                new IntakeJogCmd( intakeSubsystem, false ).withTimeout(3.0),
                StowCmdHigh()
            )
        );*/

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command autoBuilderCommand = autoBuilder.fullAuto(pathGroup);

        return new SequentialCommandGroup(
            new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) ),
            new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
            ScoreHighProCmd().withTimeout(3.5),
            new WaitCommand(0.5),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(0.5),
            autoBuilderCommand,
            ScoreMidProCmd(),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(1.5),
            StowCmdHigh()
        );
    }

    private Command AutoMobilityCmd(){
        // Load the PathPlanner path file and generate it with a max
        // velocity and acceleration for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mobility", 
            new PathConstraints(0.55, 2.0));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command autoBuilderCommand = autoBuilder.fullAuto(pathGroup);

        return new SequentialCommandGroup(
            new InstantCommand( () -> swerveSubsystem.zeroHeading(0.0) ),
            new InstantCommand( () -> swerveSubsystem.initialPitch() ),
            new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
            ScoreHighProCmd().withTimeout(3.5),
            new IntakeJogCmd( intakeSubsystem, false ).withTimeout(0.3),
            StowCmdHigh(),
            new WaitCommand(7.0),
            autoBuilderCommand
        );
    }
 }

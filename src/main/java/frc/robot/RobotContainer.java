// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.SignalLogger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    
    public static double INTAKE_ROLLER_SPEED_CURRENT;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      // Per TU12, Michigan and champs are both NOT k2025ReefscapeAndyMark
      .loadField(AprilTagFields.k2025Reefscape); 

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem( this );
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final VisionSubsystem m_Vision = new VisionSubsystem(drivetrain);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)*4; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    double driveDivider = Constants.DriveConstants.DRIVE_DIVIDER_NORMAL;
   
    public final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /* Path follower */
    public RobotContainer() { 
        
        NamedCommands.registerCommand("ScoreL4", ScoreL4Command());
        NamedCommands.registerCommand("Stow", StowCommand());
        NamedCommands.registerCommand("LowAlgae", LowAlgaeCommand());
        NamedCommands.registerCommand("HighAlgae", HighAlgaeCommand());
        NamedCommands.registerCommand("ProcessorOut", ProcessorOutCommand() );
        NamedCommands.registerCommand("ProcessorIntake", ProcessorIntakeCommand());
        NamedCommands.registerCommand("Intake", IntakeCommand());
        NamedCommands.registerCommand("Align", CoralAlign());

        configureBindings();

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

         autoChooser = AutoBuilder.buildAutoChooser("Tests");
         SmartDashboard.putData("Auto Mode", autoChooser);
    
    }

    private void DriveDividerSet( double divider )
    {
        if (elevatorSubsystem.getPosition() > 12)
        driveDivider = 4;
        else
        driveDivider = divider;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed/driveDivider) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed/driveDivider) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate/driveDivider) // Drive counterclockwise with negative X (left)
            )
        );

        /*driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));*/

        driverJoystick.povRight().whileTrue(CoralAlign());

        //creep forward and back, robot oriented
        /*driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /*Trigger driverButtonB = driverJoystick.povRight();
        driverButtonB
        .onTrue( Commands.runOnce(SignalLogger::start));

        Trigger driverButtonX = driverJoystick.povLeft();
        driverButtonX
        .onTrue( Commands.runOnce(SignalLogger::stop));*/


        Trigger driverLeftTrigger = driverJoystick.leftTrigger( 0.7 );
        driverLeftTrigger
            .onFalse(Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_NORMAL )))
            .onTrue( Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_SLOW )));

        Trigger driverRightTrigger = driverJoystick.rightTrigger(0.7);
            driverRightTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 )))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )));


        //Trigger driverYTrigger = driverJoystick.y();
        //driverYTrigger
          //  .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_KEEP )))
            //.onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN )));

        
        Trigger driverLeftBumperTrigger = driverJoystick.leftBumper();
        driverLeftBumperTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 )))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN ))
            .andThen(Commands.waitSeconds(.02))
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 ))));

        Trigger driverRightBumper = driverJoystick.rightBumper();
        driverRightBumper
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0)))
            .whileTrue(
                Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_IN ))
            .andThen( Commands.waitSeconds(10)
                    .until( intakeSubsystem::isIntakeBeamBreakLoaded) )
            .andThen( Commands.waitSeconds(0.05))
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0)))
            );
   
        
        Trigger operatorL4Trigger = operatorJoystick.y();
        operatorL4Trigger
         .onTrue(
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4)))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_HIGH)))
            );
            

        Trigger operatorL3Trigger = operatorJoystick.b();
        operatorL3Trigger
         .onTrue(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L3)))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_REEF))
            ));

        Trigger operatorL2Trigger = operatorJoystick.a();
            operatorL2Trigger
            .onTrue( 
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L2)))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_REEF))
            ));

            Trigger operatorL1Trigger = operatorJoystick.x();
            operatorL1Trigger
            .onTrue( 
                Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_START))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_START)))

            );

        Trigger operatorRightJoystickAxisUp = operatorJoystick.axisGreaterThan(5, 0.7 );
        operatorRightJoystickAxisUp
            .onFalse(Commands.runOnce( ()-> clawSubsystem.setClawJog( 0 ), clawSubsystem))
            .whileTrue( Commands.runOnce( ()-> clawSubsystem.setClawJog( -0.3 ), clawSubsystem));
        
        Trigger operatorRightJoystickAxisDown = operatorJoystick.axisLessThan(5, -0.7 );
        operatorRightJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> clawSubsystem.setClawJog( 0 ), clawSubsystem))
            .whileTrue( Commands.runOnce( ()-> clawSubsystem.setClawJog( 0.3 ), clawSubsystem));
        
        Trigger operatorLeftJoystickAxisUp = operatorJoystick.axisGreaterThan(1, 0.7 );
        operatorLeftJoystickAxisUp 
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), elevatorSubsystem))
            .onTrue( Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( -0.10 ), elevatorSubsystem));
        
        Trigger operatorLeftJoystickAxisDown = operatorJoystick.axisLessThan(1, -0.7 );
        operatorLeftJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), elevatorSubsystem))
            .onTrue(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0.15 ), elevatorSubsystem));

        //Algea Low
        Trigger operatorDPadleft = operatorJoystick.povLeft();
        operatorDPadleft
        .onTrue(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE))
            .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_LOW)))
        );

        //Algea High
        Trigger operatorDPadUp = operatorJoystick.povUp();
        operatorDPadUp
        .onTrue(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE))
            .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_HIGH)))
        );

        //Algea Store
        Trigger operatorDPadRight = operatorJoystick.povRight();
        operatorDPadRight
        .onTrue( 
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START))
            .andThen(Commands.waitSeconds(3)
            .until( clawSubsystem::atSetpoint))
            .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE_STORE))
        ));
        
        //Algea Out
        Trigger operatorDPadDown = operatorJoystick.povDown();
        operatorDPadDown
        .onTrue( 
            Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_PROCESSOR))
            .andThen( Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_PROCESSOR))));
        
        //Stow
        Trigger operatorBack = operatorJoystick.back();
        operatorBack
        .whileTrue(
            Commands.runOnce( ()->System.out.println("Stow Sequence") ) 
            .andThen( 
                Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
            .andThen(Commands.waitSeconds(3)
            .until( clawSubsystem::atSetpoint))
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
            .andThen(Commands.waitSeconds(3)
            .until(elevatorSubsystem::isAtPosition))
            .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem))
            /* .andThen(
                //Commands.runOnce( ()->setBling(0, 255, 0) ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) )
            )*/
            .andThen(Commands.waitSeconds(0.5))
            .andThen(//Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                //Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->StopControls(true) )
            )
        );

        Trigger operatorRightTrigger = operatorJoystick.rightTrigger(0.7);
        operatorRightTrigger
        .whileTrue(
            Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem)
            .andThen(Commands.waitSeconds(3)
            .until( clawSubsystem::atSetpoint))
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLIMB), elevatorSubsystem))
        );

        Trigger operatorLeftTrigger = operatorJoystick.leftTrigger(0.7);
        operatorLeftTrigger
        .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setCurrentNormal()))
        .whileTrue(
            Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem)
            .andThen(Commands.waitSeconds(3)
            .until( clawSubsystem::atSetpoint))
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START), elevatorSubsystem))
        );

        //Trigger driverClimb = driverJoystick.x();
        //driverClimb
       // .whileTrue( Commands.runOnce( ()-> elevatorSubsystem.setCurrentClimb(ClimbConstants.CLIMB_CURRENT)))
        //.onFalse(Commands.runOnce( ()-> elevatorSubsystem.setCurrentClimb(ClimbConstants.MATCH_CURRENT)));


        Trigger operatorLeftBumper = operatorJoystick.leftBumper();
        operatorLeftBumper
        .onTrue( 
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE_TRAVEL))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_BARGE))
                .andThen(Commands.waitSeconds(3)
                .until( elevatorSubsystem::isAtPosition))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_BARGE)))
            ));
    }
    public Command ScoreL4Command() 
    {
        return Commands.runOnce( ()->System.out.println("ScoreL4Command") )
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))) 
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4)))
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_REEF)))
        .andThen(Commands.waitSeconds(1.5)
        .until(elevatorSubsystem::isAtPosition))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )))              
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4 + 1.25)))
        .andThen(Commands.waitSeconds(.5)
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0 ))))              
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until(elevatorSubsystem::isAtPosition))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem))
        .andThen(Commands.waitSeconds(0.02))
        .andThen(Commands.runOnce( ()->StopControls(true))) 
        ;   
    }
    public Command LowAlgaeCommand(){

        return Commands.runOnce( ()->System.out.println("LowAlgae") )
        .andThen(Commands.waitSeconds(.5))
        .andThen(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE)))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_LOW))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN ))))              
        .andThen(Commands.waitSeconds(2.5))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_KEEP)))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ALGAE_STORE), clawSubsystem))
        .andThen(Commands.waitSeconds(2)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.runOnce( ()->StopControls(true) )
        );      
    }
    public Command HighAlgaeCommand(){

        return Commands.runOnce( ()->System.out.println("HighAlgae") )
        .andThen(Commands.waitSeconds(.5))
        .andThen(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE)))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_HIGH))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT ))))              
        .andThen(Commands.waitSeconds(2))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0 )))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ALGAE_STORE), clawSubsystem))
        .andThen(Commands.waitSeconds(2
        )
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.runOnce( ()->StopControls(true) )
        );       
    }

    public Command ProcessorOutCommand(){
    return Commands.runOnce( ()->System.out.println("ProcessorOut") )
    .andThen(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_PROCESSOR))
    .andThen( Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_PROCESSOR))));
    }

    public Command ProcessorIntakeCommand(){
        return Commands.runOnce( ()->System.out.println("ProcessorIntake") )
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )))            
        .andThen(Commands.waitSeconds(2))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0)));
    }

    public Command IntakeCommand(){
        return Commands.runOnce( ()->System.out.println("IntakeCommand") )
        .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_IN ))
        .andThen( Commands.waitSeconds(10)
                .until( intakeSubsystem::isIntakeBeamBreakLoaded) )
        .andThen( Commands.waitSeconds(0.05))
        .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0)))
        );

    }

    public Command StowCommand(){
        return Commands.runOnce( ()->System.out.println("Stow") )
        .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0)))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until(elevatorSubsystem::isAtPosition))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem))
        .andThen(Commands.runOnce( ()->StopControls(true))
        );
    }

      public Command driveToPose(Pose2d poseStart, Pose2d poseShort, Pose2d poseFinal) 
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        2, // velocity limit
        1, // acceleration limit
        Units.degreesToRadians(360), Units.degreesToRadians(360)  // turn velocity + acceleration limits
      );

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( poseStart, poseShort, poseFinal );

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath
          (
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState( 0.0 /* velocity */, poseFinal.getRotation() )
          );

    // Prevent the path from being flipped since the coordinates are already correct
    path.preventFlipping = true;

    return AutoBuilder.followPath( path );
  }

  public Pose2d closestAprilTag(Pose2d robotPose) {
    // Use the robot pose and return the closest AprilTag on a REEF
    List<Integer> tagIDs = List.of( 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11);

    double minDistance = Double.MAX_VALUE;
    int closestTagID = 0;
    Pose2d closestTagPose = new Pose2d();

    // Loop through the reef tags and find the closest one
    for (int tagID : tagIDs) {
      var tagPoseOptional = aprilTagFieldLayout.getTagPose(tagID);
      var tagPose = tagPoseOptional.get();
      Pose2d tagPose2d = new Pose2d(tagPose.getX(), tagPose.getY(), new Rotation2d(tagPose.getRotation().getZ()));
      double distance = robotPose.getTranslation().getDistance(tagPose2d.getTranslation());

      // Remember the shortest distance in the list
      if (distance < minDistance) 
      {
        minDistance = distance;
        closestTagPose = tagPose2d;
        closestTagID = tagID;
      }
    }

    return closestTagPose;
  }

  public Command AlignCommand()
  {
    double coralOffsetDirection = 1.0;  // handles going for left side coral (+y) or right side coral (-y)
    RawFiducial fiducial;

    Pose2d closestTagPose = closestAprilTag(drivetrain.getState().Pose);
    // SmartDashboard.putNumber("Closest Tag X", closestTagPose.getX());
    // SmartDashboard.putNumber("Closest Tag Y", closestTagPose.getY());

    double x1 = closestTagPose.getX();
    double y1 = closestTagPose.getY();
    double z1 = closestTagPose.getRotation().getRadians();

    // The short position has the robot bumpers a short distance away from the reef wall
    double translatedShortX = x1 + (((reefAlignmentConstants.robotWidth / 2) + reefAlignmentConstants.shortDistance) * Math.cos(z1));
    double translatedShortY = y1 + ((reefAlignmentConstants.robotWidth / 2) * Math.sin(z1));
    double translatedRot = z1 - Math.PI;

    // The final position has the robot bumpers flush with the reef
    double translatedFinalX = x1 + ((reefAlignmentConstants.robotWidth / 2) * Math.cos(z1));
    double translatedFinalY = y1 + ((reefAlignmentConstants.robotWidth / 2) * Math.sin(z1));

    // This function will align to the left reef post if the robot is to the left of the tag,
    // or to the right reef post if the robot is to the right of the tag.
    try
    {
      fiducial = m_Vision.getFiducialWithId(m_Vision.getClosestFiducial().id);
      // If limelight TX is positive, the robot is to the right of the tag
      if( fiducial.txnc > 0 )
        coralOffsetDirection = -1.0;

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      // +X forward, +Y up
      translatedShortX += ((reefAlignmentConstants.reefSpacing - reefAlignmentConstants.coralScoreOffset) * coralOffsetDirection)
          * Math.cos(z1 + Math.PI / 2);
      translatedShortY += ((reefAlignmentConstants.reefSpacing - reefAlignmentConstants.coralScoreOffset) * coralOffsetDirection)
          * Math.sin(z1 + Math.PI / 2);

      translatedFinalX += ((reefAlignmentConstants.reefSpacing - reefAlignmentConstants.coralScoreOffset) * coralOffsetDirection)
          * Math.cos(z1 + Math.PI / 2);
      translatedFinalY += ((reefAlignmentConstants.reefSpacing - reefAlignmentConstants.coralScoreOffset) * coralOffsetDirection)
          * Math.sin(z1 + Math.PI / 2);
        
      return driveToPose( 
                          (drivetrain.getState().Pose),
                          new Pose2d(translatedShortX, translatedShortY, new Rotation2d(translatedRot)),
                          new Pose2d(translatedFinalX, translatedFinalY, new Rotation2d(translatedRot))
                        );
    }
    catch (VisionSubsystem.NoSuchTargetException nste)
    {
      // if no AprilTag is visible, just don't do anything
      return Commands.waitSeconds(15);
    }
  }

  public DeferredCommand CoralAlign () {
    return (new DeferredCommand(() -> AlignCommand(), Set.of(drivetrain)));

}

    public void StopControls( boolean stopped)
    {
        System.out.println("StopControls");
        elevatorSubsystem.setElevatorJog(0);
        clawSubsystem.setClawJog(0);
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}
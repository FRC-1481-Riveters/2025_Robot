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
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.commands.PositionPIDCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.text.DecimalFormat;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    
    public static double INTAKE_ROLLER_SPEED_CURRENT;
    public static Pose2d PreviousTagPose;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      // Per TU12, Michigan and champs are both welded, NOT k2025ReefscapeAndyMark
      .loadField(AprilTagFields.k2025ReefscapeWelded); 

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)*4; // 3/4 of a rotation per second max angular velocity  
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem( this );
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final Telemetry logger = new Telemetry(MaxSpeed, drivetrain);
    private final VisionSubsystem m_Vision = new VisionSubsystem(drivetrain);
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    double driveDivider = Constants.DriveConstants.DRIVE_DIVIDER_NORMAL;
   
    public final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /* Path follower */
    public RobotContainer() { 
        
        // hey next year make these "cmdXxxx" so it's obvious in PathPlanner that they are commands not paths
        NamedCommands.registerCommand("ScoreL4", ScoreL4Command());
        NamedCommands.registerCommand("ScoreL2", ScoreL2Command());
        NamedCommands.registerCommand("Stow", StowCommand());
        NamedCommands.registerCommand("LowAlgae", LowAlgaeCommand());
        NamedCommands.registerCommand("HighAlgae", HighAlgaeCommand());
        NamedCommands.registerCommand("ProcessorOut", ProcessorOutCommand() );
        NamedCommands.registerCommand("ProcessorIntake", ProcessorIntakeCommand());
        NamedCommands.registerCommand("Intake", IntakeCommand());
        NamedCommands.registerCommand("Align", CoralAlign());
        NamedCommands.registerCommand("Algae", AlgaeAlign());
        NamedCommands.registerCommand("MoveL4", MoveL4Command());
        NamedCommands.registerCommand("BargeShot", BargeShot());
        NamedCommands.registerCommand("PINGPONG", PingPong());
        configureBindings();

        for (int port = 5801; port <= 5809; port++) {
//            PortForwarder.add(port, "limelight-riveter.local", port);
            PortForwarder.add(port, "10.14.81.11", port);
        }

        // add limelight 3a
        PortForwarder.add(5811, "10.14.81.12", 5801);
        PortForwarder.add(5812, "10.14.81.12", 5802);
        PortForwarder.add(5813, "10.14.81.12", 5803);
        PortForwarder.add(5814, "10.14.81.12", 5804);
        PortForwarder.add(5815, "10.14.81.12", 5805);
        PortForwarder.add(5816, "10.14.81.12", 5806);
        PortForwarder.add(5817, "10.14.81.12", 5807);
        PortForwarder.add(5818, "10.14.81.12", 5808);
        PortForwarder.add(5819, "10.14.81.12", 5809);

         autoChooser = AutoBuilder.buildAutoChooser("Tests");
         SmartDashboard.putData("Auto Mode", autoChooser);
    
    }

    private void DriveDividerSet( double divider )
    {
        if (elevatorSubsystem.getPosition() > 15)
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

        driverJoystick.povRight().whileTrue(AlgaeAlign());
        driverJoystick.povLeft().whileTrue(CoralAlign());

        //creep forward and back, robot oriented
        driverJoystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(1.0).withVelocityY(0))
        );
        driverJoystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press
        driverJoystick.a().onTrue( drivetrain.runOnce( () -> drivetrain.seedFieldCentric() ) );

        drivetrain.registerTelemetry(logger::telemeterize);

        /*Trigger driverButtonB = driverJoystick.povRight();
        driverButtonB
        .onTrue( Commands.runOnce(SignalLogger::start));

        Trigger driverButtonX = driverJoystick.povLeft(); )
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


        Trigger driverYTrigger = driverJoystick.y();
        driverYTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_KEEP )))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN )));

        
        Trigger driverLeftBumperTrigger = driverJoystick.leftBumper();
        driverLeftBumperTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 )))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN ))
            .andThen(Commands.waitSeconds(.04))
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
   
        Trigger troughButton = driverJoystick.b();
        troughButton
        .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0)))
        .whileTrue(
            Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_TROUGH))
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
         .onTrue(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_HIGH))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
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
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_LOW))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L1))
            ));

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
        Trigger operatorDPadDown = operatorJoystick.povDown();
        operatorDPadDown
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
        Trigger operatorDPadLeft = operatorJoystick.povLeft();
        operatorDPadLeft
        .onTrue( 
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START))
            .andThen(Commands.waitSeconds(3)
            .until( clawSubsystem::atSetpoint))
            .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE_STORE))
        ));
        
        //Algea Out
        Trigger operatorDPadRight = operatorJoystick.povRight();
        operatorDPadRight
        .onTrue( 
            Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_PROCESSOR))
            .andThen( Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_PROCESSOR))));
        
        //Stow
        Trigger operatorBack = operatorJoystick.back();
        operatorBack
        .whileTrue(
            Commands.runOnce( ()->System.out.println("Stow Sequence") ) 
            .andThen( Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
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

        Trigger operatorLeftTrigger = operatorJoystick.leftTrigger(0.7) .and(driverJoystick.x());
        operatorLeftTrigger
        .whileTrue(
            Commands.runOnce( ()-> climbSubsystem.DeployClimb(ClimbConstants.DEPLOY_SPEED))
            .andThen(Commands.waitSeconds(10)
            .until(climbSubsystem::DeployDown))
            .andThen(
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) )
            )
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0))
            )
        )

        .onFalse(
            Commands.runOnce( ()-> climbSubsystem.DeployClimb(0)) 
        );


        Trigger operatorRightTrigger = operatorJoystick.rightTrigger(0.7) .and(driverJoystick.x());
        operatorRightTrigger
        .whileTrue(
            Commands.runOnce( ()-> climbSubsystem.ClimbClimb(ClimbConstants.CLIMB_SPEED))
            .andThen( ()->clawSubsystem.setClaw(ClawConstants.CLAW_CLIMB_CLEAR))
        )
        .onFalse(
            Commands.runOnce( ()-> climbSubsystem.ClimbClimb(0)) 
        );

        Trigger DeployRest = operatorJoystick.start();
        DeployRest
        .whileTrue(
            Commands.runOnce( ()-> climbSubsystem.DeployClimb(-ClimbConstants.DEPLOY_SPEED))
        )
        .onFalse(
            Commands.runOnce( ()-> climbSubsystem.DeployClimb(0)) 
        );

        //barge shot
        Trigger operatorLeftBumper = operatorJoystick.leftBumper();
        operatorLeftBumper
        .onTrue( BargeShot() )
        .onFalse(
            Commands.runOnce(()-> intakeSubsystem.setIntakeRollerSpeed(0))
        );
    }

    public Command BargeShot()
    {
        return Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE_TRAVEL-1.5))//Constants.ClawConstants.CLAW_ALGAE_STORE + 4))
            .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_BARGE))
            .alongWith(Commands.waitSeconds(.7))
            .andThen(Commands.runOnce(()-> intakeSubsystem.setIntakeRollerSpeed(0))))
            .andThen(Commands.waitSeconds(5)
            .until(elevatorSubsystem::PastBarge))
            .andThen(Commands.runOnce(()-> clawSubsystem.setClawPidClamp(Constants.ClawConstants.CLAW_PID_CLAMP_HIGH)))
            .andThen(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_BARGE)))
            .andThen(Commands.waitSeconds(5)
            .until(clawSubsystem::PastFlick))
            .andThen(Commands.runOnce(()-> intakeSubsystem.setIntakeRollerSpeed(Constants.IntakeConstants.INTAKE_ROLLER_SPEED_BARGE)))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(Commands.runOnce(()-> clawSubsystem.setClawPidClamp(Constants.ClawConstants.CLAW_PID_CLAMP_NORMAL)))
            .andThen(Commands.runOnce(()-> intakeSubsystem.setIntakeRollerSpeed(0))
        );
    }
    public Command ScoreL4Command() 
    {
        return Commands.runOnce( ()->System.out.println("ScoreL4Command") )
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_AUTON_CLEAR))) 
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4)))
        .andThen(Commands.waitSeconds(3)
        .until( elevatorSubsystem::isAtPosition))
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_HIGH)))
        .andThen(Commands.waitSeconds(1.5)
        .until(elevatorSubsystem::isAtPosition))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )))              
        .andThen(Commands.waitSeconds(0.5))
        //.andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4 + 1.25)))
        .andThen(Commands.waitSeconds(2)
        .until(intakeSubsystem::isIntakeBeamBreakOut))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0 )))            
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        ;   
    }

    public Command MoveL4Command() 
    {
        return Commands.runOnce( ()->System.out.println("MoveL4Command") )
        .andThen(Commands.waitSeconds(0.75))
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_AUTON_CLEAR))) 
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4)))
        .andThen(Commands.waitSeconds(3)
        .until( elevatorSubsystem::isAtPosition));
    }

    public Command ScoreL2Command() 
    {
        return Commands.runOnce( ()->System.out.println("ScoreL4Command") )
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))) 
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L2)))
        .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_REEF)))
        .andThen(Commands.waitSeconds(1.5)
        .until(elevatorSubsystem::isAtPosition))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )))              
        //.andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4 + 1.25)))
        .andThen(Commands.waitSeconds(.5)
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0 ))))              
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until(elevatorSubsystem::isAtPosition))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(Commands.runOnce( ()->StopControls(true))) 
        ;   
    }

    public Command LowAlgaeCommand(){

        return Commands.runOnce( ()->System.out.println("LowAlgae") )
        .andThen(Commands.waitSeconds(1.0))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_KEEP)))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ALGAE_STORE), clawSubsystem))
        .andThen(Commands.waitSeconds(2)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
       // .andThen(Commands.runOnce( ()->StopControls(true) )
        ;      
    }
    public Command HighAlgaeCommand(){

        return Commands.runOnce( ()->System.out.println("HighAlgae") )
        .andThen(Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE)))
        .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_HIGH))
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT ))))              
        .andThen(Commands.waitSeconds(1.5))
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
        //.andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0)))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until(elevatorSubsystem::isAtPosition))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem)
        //.andThen(Commands.runOnce( ()->StopControls(true))
        );
    }
   
    public Command driveToPose(Pose2d poseStart, Pose2d poseShort, Pose2d poseFinal) 
    {
/*  use PositionPIDCommand instead of PathPlanner for the time being
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        0.5, // velocity limit
        .5, // acceleration limit
        Units.degreesToRadians(90), Units.degreesToRadians(90)  // turn velocity + acceleration limits
        );

     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( poseStart, poseShort );

    // Create the path using the waypoints created above
    /*PathPlannerPath path = new PathPlannerPath
            ( waypoints, constraints, null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState( 0.0  velocity , poseShort.getRotation() )
            );
            

    // Prevent the path from being flipped since the coordinates are already correct
    path.preventFlipping = true;
    return( AutoBuilder.followPath( path )
        .andThen( PositionPIDCommand.generateCommand( drivetrain, poseFinal, 15) ));
    }
*/

    return (PositionPIDCommand.generateCommand(drivetrain, poseFinal, 4.0));
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
      Pose2d tagPose2d = new Pose2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation().toRotation2d());
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

  public Command CoralAlignCommand()
  {
    double coralOffsetDirection = -1.0;  // handles going for left side coral (+y) or right side coral (-y)
    RawFiducial fiducial;

    Pose2d botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-riveter").pose;
    if( botPose.getX() != 0 )
    {
        drivetrain.resetPose( botPose );
    }

    Pose2d closestTagPose = closestAprilTag(drivetrain.getState().Pose);
    PreviousTagPose = closestTagPose;

    // This function will align to the left reef post if the robot is to the left of the tag,
    // or to the right reef post if the robot is to the right of the tag.
    try
    {
        fiducial = m_Vision.getFiducialWithId(m_Vision.getClosestFiducial().id);
        // If your target is on the rightmost edge of 
        // your limelight feed, tx should return roughly 31 degrees.
        // If the robot is aimed vaguely towards the reef, and the target is on the right, txnc will be positive
        if( fiducial.txnc < 0 )
            coralOffsetDirection = 1.0;

        // Make a Transform2d to calculate the offset of the robot position 
        // when it's up against the edge of the reef, lined up with the
        // correct coral post.
        // The offset includes reefSpacing (distance between coral posts)
        // and the bumper-to-bumper width of the robot itself
        Transform2d coralOffsetLeft = new Transform2d( 
            reefAlignmentConstants.robotWidth / 2, 
            coralOffsetDirection * reefAlignmentConstants.reefSpacing/2 + reefAlignmentConstants.coralScoreOffset, 
            Rotation2d.kZero );

        // Make a Transform2d to calculate the offset of the robot position 
        // when it's up against the edge of the reef, lined up with the
        // correct coral post.
        // The offset includes reefSpacing (distance between coral posts)
        // and the bumper-to-bumper width of the robot itself, AND
        // a short distance where PositionPIDCommand is used instead of
        // PathPlanner, because PathPlanner is only accurate to +/- 2".
        Transform2d coralOffsetLeftShort = new Transform2d( 
            reefAlignmentConstants.robotWidth / 2 + reefAlignmentConstants.shortDistance, 
            coralOffsetDirection * reefAlignmentConstants.reefSpacing/2 + reefAlignmentConstants.coralScoreOffset,
            Rotation2d.kZero );

        Pose2d tagReefEdgePose = closestTagPose.plus(coralOffsetLeft);
        Pose2d tagReefShortPose = closestTagPose.plus( coralOffsetLeftShort );

        // AprilTag poses are from the face of the tag (out from the reef)
        // Convert the poses to robot poses FACING the reef
        Pose2d robotReefEdgePose = new Pose2d( tagReefEdgePose.getX(), tagReefEdgePose.getY(), tagReefEdgePose.getRotation().plus(Rotation2d.kPi));
        Pose2d robotReefShortPose = new Pose2d( tagReefShortPose.getX(), tagReefShortPose.getY(), tagReefShortPose.getRotation().plus(Rotation2d.kPi));

        return driveToPose( drivetrain.getState().Pose, robotReefShortPose, robotReefEdgePose );
    }
    catch (VisionSubsystem.NoSuchTargetException nste)
    {
        // if no AprilTag is visible, just don't do anything
        System.out.println("Align: no tag is visible");
        return Commands.waitSeconds(3);
    }
}
                
public Command PingPongCommand()
{
  Pose2d targetLoader = new Pose2d(1.211,7.028, Rotation2d.fromDegrees(-45));
  Pose2d targetReef = new Pose2d(3.836, 5.194, Rotation2d.fromDegrees(-61.091));

  return driveToPose( drivetrain.getState().Pose, targetReef, targetReef )
    .andThen( driveToPose( drivetrain.getState().Pose, targetLoader, targetLoader ) )
    .andThen( driveToPose( drivetrain.getState().Pose, targetReef, targetReef ) )
    .andThen( driveToPose( drivetrain.getState().Pose, targetLoader, targetLoader ) );
}

public DeferredCommand CoralAlign () {
        return (new DeferredCommand(() -> CoralAlignCommand(), Set.of(drivetrain)));
    }

    public DeferredCommand PingPong () {
        return (new DeferredCommand(() -> PingPongCommand(), Set.of(drivetrain)));
    }

  public Command AlgaeAlignCommand()//boolean useLimeLight)
  {
    RawFiducial fiducial;
    Pose2d closestTagPose;

    Pose2d botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-riveter").pose;
    if( botPose.getX() != 0 )
    {
        drivetrain.resetPose( botPose );
    }


    //if(useLimeLight)
    //{
    closestTagPose = closestAprilTag(drivetrain.getState().Pose);
    PreviousTagPose = closestTagPose;
   // }
    //else
    //closestTagPose = PreviousTagPose;

    // Make a Transform2d to calculate the offset of the robot position 
    // when it's up against the edge of the reef, lined up with the
    // correct coral post.
    // The offset includes reefSpacing (distance between coral posts)
    // and the bumper-to-bumper width of the robot itself
    Transform2d robotOffset = new Transform2d( 
        reefAlignmentConstants.robotWidth / 2 - 0.1, 
        0,
        Rotation2d.kZero );

    // Make a Transform2d to calculate the offset of the robot position 
    // when it's up against the edge of the reef, lined up with the
    // correct coral post.
    // The offset includes reefSpacing (distance between coral posts)
    // and the bumper-to-bumper width of the robot itself, AND
    // a short distance where PositionPIDCommand is used instead of
    // PathPlanner, because PathPlanner is only accurate to +/- 2".
    Transform2d robotOffsetShort = new Transform2d( 
        reefAlignmentConstants.robotWidth / 2 + reefAlignmentConstants.shortDistance, 
        0,
        Rotation2d.kZero );

    // This function will align to the left reef post if the robot is to the left of the tag,
    // or to the right reef post if the robot is to the right of the tag.
    try
    {
        fiducial = m_Vision.getFiducialWithId(m_Vision.getClosestFiducial().id);
        // If your target is on the rightmost edge of 
        // your limelight feed, tx should return roughly 31 degrees.
        // If the robot is aimed vaguely towards the reef, and the target is on the right, txnc will be positive

        Pose2d tagReefEdgePose = closestTagPose.plus(robotOffset);
        Pose2d tagReefShortPose = closestTagPose.plus(robotOffsetShort);

        // AprilTag poses are from the face of the tag (out from the reef)
        // Convert the poses to robot poses FACING the reef
        Pose2d robotReefEdgePose = new Pose2d( tagReefEdgePose.getX(), tagReefEdgePose.getY(), tagReefEdgePose.getRotation().plus(Rotation2d.kPi));
        Pose2d robotReefShortPose = new Pose2d( tagReefShortPose.getX(), tagReefShortPose.getY(), tagReefShortPose.getRotation().plus(Rotation2d.kPi));

//        return driveToPose( drivetrain.getState().Pose, robotReefShortPose, robotReefEdgePose);
        return Commands.runOnce( ()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ALGAE) )
            .alongWith( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_LOW) ) )
            .alongWith( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE_IN )) )
            .alongWith( driveToPose( drivetrain.getState().Pose, robotReefShortPose, robotReefEdgePose) )
            .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ALGAE_STORE), clawSubsystem)
            );
    }
    catch (VisionSubsystem.NoSuchTargetException nste)
    {
      // if no AprilTag is visible, just don't do anything
      System.out.println("Align: no tag is visible");
      return Commands.waitSeconds(3);
    }
  }

  public DeferredCommand AlgaeAlign (){//boolean useLimeLight) {
    return (new DeferredCommand(() -> AlgaeAlignCommand(), Set.of(drivetrain)));

    }

    public void StopControls( boolean stopped)
    {
        System.out.println("StopControls");
        elevatorSubsystem.setElevatorJog(0);
        clawSubsystem.setClawJog(0);
    }


    public Command getAutonomousCommand() {
        Command command;
        command = autoChooser.getSelected();
        String name = command.getName();
        if( name.startsWith("right ") )
        {
            // if the path starts with "right ", mirror it from a left path
            // i.e., name the left path "2 coral us", and make a dummy right path "right 2 coral us"
            // - this will skip the dummy path and mirror the left path instead
            System.out.println("Flipping auton path " + name.substring(6));
            command = new PathPlannerAuto( name.substring(6), true );
        }
        else
        {
            System.out.println("Using auton path " + name);
        }
        return command;
    }

    public void testReefPositions()
    {
        int[] reefAprilTags = new int[] {6,7,8,9,10,11,17,18,19,20,21,22};
        for( int i=0; i < reefAprilTags.length; i++  )
        {
            var tagPoseOptional = aprilTagFieldLayout.getTagPose(reefAprilTags[i]);
            var tagPose = tagPoseOptional.get();
            Pose2d tagPose2d = new Pose2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation().toRotation2d());

            // Make a Transform2d to calculate the offset
            Transform2d coralOffsetLeft = new Transform2d( 
                reefAlignmentConstants.robotWidth / 2, 
                -1 * reefAlignmentConstants.reefSpacing/2, 
                Rotation2d.kZero );

            Transform2d coralOffsetLeftShort = new Transform2d( 
                reefAlignmentConstants.robotWidth / 2 + reefAlignmentConstants.shortDistance, 
                -1 * reefAlignmentConstants.reefSpacing/2,
                Rotation2d.kZero );

            Pose2d tagReefEdgePose = tagPose2d.plus(coralOffsetLeft);
            Pose2d tagReefShortPose = tagPose2d.plus( coralOffsetLeftShort );

            // AprilTag poses are from the face of the tag (out from the reef)
            // Convert the poses to robot poses FACING the reef
            Pose2d robotReefEdgePose = new Pose2d( tagReefEdgePose.getX(), tagReefEdgePose.getY(), tagReefEdgePose.getRotation().plus(Rotation2d.kPi));
            Pose2d robotReefShortPose = new Pose2d( tagReefShortPose.getX(), tagReefShortPose.getY(), tagReefShortPose.getRotation().plus(Rotation2d.kPi));

            System.out.println("==== TAG " + reefAprilTags[i] + " ====");
            DecimalFormat df = new DecimalFormat( "#.00");
            System.out.println("reef x=" + df.format(tagPose2d.getX()) + "\treef y=" + df.format(tagPose2d.getY()) + "\treef rotate=" + df.format(tagPose2d.getRotation().getDegrees()));
            System.out.println("left x=" + df.format(robotReefEdgePose.getX()) + "\tleft y=" + df.format(robotReefEdgePose.getY()) + "\tleft rotate=" + df.format(robotReefEdgePose.getRotation().getDegrees()));
            System.out.println("shrt x=" + df.format(robotReefShortPose.getX()) + "\tshrt y=" + df.format(robotReefShortPose.getY()) + "\tshrt rotate=" + df.format(robotReefShortPose.getRotation().getDegrees()));
        }
    }
}
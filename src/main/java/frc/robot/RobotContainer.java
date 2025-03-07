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
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AlignSlide;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.*;
import frc.robot.commands.AlignCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    
    public static double INTAKE_ROLLER_SPEED_CURRENT;
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem( this );
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final VisionSubsystem m_Vision = new VisionSubsystem();
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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    public RobotContainer() { 
        
        NamedCommands.registerCommand("ScoreL4", ScoreL4Command());
        NamedCommands.registerCommand("Stow", StowCommand());
        NamedCommands.registerCommand("LowAlgae", LowAlgaeCommand());
        NamedCommands.registerCommand("HighAlgae", HighAlgaeCommand());
        NamedCommands.registerCommand("ProcessorOut", ProcessorOutCommand() );
        NamedCommands.registerCommand("ProcessorIntake", ProcessorIntakeCommand());

        configureBindings();

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        NamedCommands.registerCommand("Align", 
        new AlignCommand(drivetrain, m_Vision).withTimeout(2));

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

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));*/

        /*driverJoystick.x().whileTrue(new AlignCommand(drivetrain, m_Vision));
        driverJoystick.b().onTrue(new AlignSlide(drivetrain, 1, 1, 0.5));*/

        driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        Trigger driverButtonB = driverJoystick.povRight();
        driverButtonB
        .onTrue( Commands.runOnce(SignalLogger::start));

        Trigger driverButtonX = driverJoystick.povLeft();
        driverButtonX
        .onTrue( Commands.runOnce(SignalLogger::stop));


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
            .andThen(Commands.waitSeconds(.02))
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 ))));
   
        
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

        Trigger driverClimb = driverJoystick.x();
        driverClimb
        .whileTrue( Commands.runOnce( ()-> elevatorSubsystem.setCurrentClimb(ClimbConstants.CLIMB_CURRENT)))
        .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setCurrentClimb(ClimbConstants.MATCH_CURRENT)));


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
        .andThen(Commands.waitSeconds(.5)
        .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed(0 ))))              
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_ELEVATOR_CLEAR), clawSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until( clawSubsystem::atSetpoint))
        .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem))
        .andThen(Commands.waitSeconds(3)
        .until(elevatorSubsystem::isAtPosition))
        .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_START), clawSubsystem))
        .andThen(Commands.runOnce( ()->StopControls(true) )
        );   
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
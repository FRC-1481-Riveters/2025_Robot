package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class RobotContainer 
{
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem( this );
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem( elevatorSubsystem );
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    public final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_autoChooser;


    double driveDivider = Constants.DriveConstants.DRIVE_DIVIDER_NORMAL;

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    public CANdle m_CANdle;
    public boolean m_allTestsPassed;

    public RobotContainer() 
    {
        m_led = new AddressableLED(0);
        m_CANdle = new CANdle(Constants.OIConstants.CANDLE_ID);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(36);
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.1;
        m_CANdle.configAllSettings(configAll, 100);
        m_CANdle.animate( new RainbowAnimation(1.0, 1.0, 8), 0 );
    

        configureButtonBindings();

        // Register named pathplanner commands
        /*NamedCommands.registerCommand("ShootCommand", AutonShooterCommand());
        NamedCommands.registerCommand("ShootAgainCommand", AutonShootAgainCommand());
        NamedCommands.registerCommand("Shoot3FootCommand", AutonShooter3FootCommand());
        NamedCommands.registerCommand("SpewCommand", AutonSpewCommand());
        NamedCommands.registerCommand("IntakeRetractCommand", IntakeRetractAutoCommand() );
        NamedCommands.registerCommand("IntakeDeployCommand", IntakeDeployAutoCommand() );
        NamedCommands.registerCommand("IntakeRollersIn", IntakeRollersInCommand() );
        NamedCommands.registerCommand("IntakeRollersStop", IntakeRollersStopCommand() );
        NamedCommands.registerCommand("Stow", AutonStowCommand());*/
        // A chooser for autonomous commands
        // Add a button to run the example auto to SmartDashboard
        //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
        m_autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData( "Auto Mode", m_autoChooser ); 
    }

    public void setRosie()
    {
        int i;
        for( i=0; i<m_ledBuffer.getLength(); i += 3 )
        {
            m_ledBuffer.setRGB(i+0, 255, 0, 0);
            m_ledBuffer.setRGB(i+1, 255, 0, 0);;
            m_ledBuffer.setRGB(i+2, 255, 255, 255);
        }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        m_CANdle.clearAnimation(0);
        for( i=0; i<8; i++ )
        {
            if( i % 2 == 0 )
                m_CANdle.setLEDs(255,0,0,0,i,1);
            else
                m_CANdle.setLEDs(255,255,255,255,i,1);
        }
    }

    public void setBling( int red, int green, int blue )
    {
        // limelight ledMode: 1=off, 2=blink, 3=on
        //if(red == 0 && blue == 0 && green == 255){
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        //}else if(red == 255 && green == 255 && blue == 0 ){
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
        //}else if(red == 0 && green == 0 && blue == 0){
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        //}
        //else if( red == 255 && green == 25 && blue == 0 ) {
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
            //schedule brief double rumble 
            //GamepieceRumbleCommand().schedule();
        //}

        int i;
        for( i=0; i<m_ledBuffer.getLength(); i++ )
        {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        m_CANdle.clearAnimation(0);
        m_CANdle.setLEDs(red,green,blue,0,0,8);
    }

    private double getDriverMoveFwdBack()
    {
        double pos;
        // Use the joystick axis
        pos = driverJoystick.getRawAxis(OIConstants.kDriverYAxis) / driveDivider;
        return pos;
    }

    private double getDriverMoveLeftRight()
    {
        double pos;
        pos = driverJoystick.getRawAxis(OIConstants.kDriverXAxis) / driveDivider;
        return pos;
    }

    private double getDriverRotate()
    {
        double pos;
        pos = -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis) / driveDivider;
        return pos;
    }

    private void DriveDividerSet( double divider )
    {
        driveDivider = divider;
    }
    
    private void configureButtonBindings() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

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
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        Trigger driverLeftTrigger = driverJoystick.leftTrigger( 0.7 );
        driverLeftTrigger
            .onFalse(Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_NORMAL )))
            .onTrue( Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_TURBO )));

        Trigger driverRightTrigger = driverJoystick.rightTrigger(0.7);
        driverRightTrigger
        .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( 0 )))
        .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_CORAL_OUT )));


        Trigger driverLeftBumper = driverJoystick.leftBumper();
        driverLeftBumper
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_KEEP )))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRollerSpeed( Constants.IntakeConstants.INTAKE_ROLLER_SPEED_ALGAE )));

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
            
        Trigger startButton = operatorJoystick.start();
        startButton
            .onTrue( Commands.runOnce( () -> elevatorSubsystem.zeroEncoder() ) );


        Trigger operatorL4Trigger = operatorJoystick.y();
        operatorL4Trigger
         .onTrue( 
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))
                .andThen(Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint))
                .andThen(Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4)))
                .andThen( Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_REEF))
            ));

        Trigger operatorL3Trigger = operatorJoystick.b();
        operatorL3Trigger
         .onTrue( 
                Commands.runOnce(()-> clawSubsystem.setClaw(Constants.ClawConstants.CLAW_ELEVATOR_CLEAR))
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
            .andThen(
                Commands.runOnce( ()->setBling(0, 255, 0) ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) )
            )
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->StopControls(true) )
            )
        );
    }
    
    /*public Command AutonShooterCommand() 
    {
        return Commands.runOnce( ()->System.out.println("AutonShooterCommand") )
            .andThen(Commands.runOnce(()-> swerveSubsystem.saveOdometry()))
            /*.andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PIVOT_CLEAR), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            ) 
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(0), elevatorSubsystem)),
                Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_REEF), clawSubsystem)
            )
            .andThen( Commands.waitSeconds(3.0)
                .until( this::isAtAllPositions ))
            .andThen( IntakeRollersOutCommand())
            .andThen( Commands.waitSeconds(0.60))
            .andThen( IntakeRollersStopCommand())
        ;
    }

    public Command AutonShootAgainCommand() 
    {
        return Commands.runOnce( ()->System.out.println("AutonShootAgainCommand") )
            .andThen(Commands.runOnce(()-> swerveSubsystem.resetOdometry(swerveSubsystem.getSavedOdometry())))
            .andThen( Commands.waitSeconds(3.0)
                .until( this::isAtAllPositions ))
            .andThen( IntakeRollersOutCommand())
            .andThen( Commands.waitSeconds(0.60))
            .andThen( IntakeRollersStopCommand())
        ;
    }

    public Command AutonSpewCommand()
    {
        return Commands.runOnce( ()->System.out.println("AutonShooterCommand") )
            .andThen(Commands.runOnce(()-> swerveSubsystem.saveOdometry()))
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(0), elevatorSubsystem)
            )
            .andThen( Commands.waitSeconds(3.0)
                .until( this::isAtAllPositions ))
            .andThen( IntakeRollersOutCommand())
            .andThen( Commands.waitSeconds(0.60))
            .andThen( IntakeRollersStopCommand());
    }

    public Command AutonShooter3FootCommand() 
    {
        return Commands.runOnce( ()->System.out.println("AutonShooter3FootCommand") )
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(0), elevatorSubsystem),
                Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_BARGE), clawSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_BARGE), elevatorSubsystem)
            )
            .andThen( Commands.waitSeconds(3.0)
                .until( this::isAtAllPositions ))
            .andThen(IntakeRollersOutCommand())
            .andThen(Commands.waitSeconds(0.60))
            .andThen(IntakeRollersStopCommand())
        ;
    }

    public Command AutonStowCommand()
    {
        return Commands.runOnce( ()->System.out.println("Auton Stow Sequence") ) 
            .andThen(Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(0), elevatorSubsystem))
            .andThen( Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint)
            )
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_BARGE ), elevatorSubsystem)
            .andThen( Commands.waitSeconds(3)
                .until( elevatorSubsystem::isAtPosition)
            )
            .andThen( 
                Commands.runOnce( ()-> clawSubsystem.setClaw(ClawConstants.CLAW_BARGE), clawSubsystem)
            )
            .andThen( Commands.waitSeconds(3)
                .until( clawSubsystem::atSetpoint)
            )
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem)
            )
            .andThen( Commands.waitSeconds(10) 
                .until( elevatorSubsystem::isAtPosition ))
            .andThen(Commands.runOnce( ()->StopControls(true) ))
            );
    }

    public Command IntakeRetractCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRetractCommand") );
          //  .andThen(Commands.waitSeconds(3))
               // .until(intakeSubsystem::atSetpoint)
                //.finallyDo( intakeSubsystem::intakeAngleDisable);
    }

    public Command IntakeDeployCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeDeployCommand") )
            .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(0), clawSubsystem));
    }

    public Command IntakeRetractAutoCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRetractAutoCommand") )
            .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(0), clawSubsystem));
    }

    public Command IntakeDeployAutoCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeDeployAutoCommand") )
            .andThen(Commands.runOnce( ()-> clawSubsystem.setClaw(0), clawSubsystem));
    }

    public Command IntakeRollersInCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRollersInCommand") )
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( -0.7 ) ) )
            .andThen( Commands.waitSeconds(20))
                //.until(intakeSubsystem::isIntakeBeamBreakLoaded))
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 )));
    }

    public Command IntakeRollersStopCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRollersStopCommand") )
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 )));
    }

    public Command IntakeRollersOutCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRollersOutCommand") )
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller(1 )));
    }

    public Command GamepieceRumbleCommand()
    {
        return 
            Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.25) )
            .andThen(
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.25) ),
                Commands.waitSeconds(0.2 ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.waitSeconds(0.2 ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.25) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.25) ),
                Commands.waitSeconds(0.2 ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) )
            );
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        return m_autoChooser.getSelected();
    }

    public void StopControls( boolean stopped)
    {
        System.out.println("StopControls");
        clawSubsystem.setClawJog(0);
        elevatorSubsystem.setElevatorJog(0);
        intakeSubsystem.setIntakeRoller( 0.0 );
    }

    public boolean isAtAllPositions()
    {
        if( elevatorSubsystem.isAtPosition() && 
            clawSubsystem.atSetpoint() )
            return true;
        else
            return false;
    }

    public void testElevatorBeamBreak(boolean intendedState){
        String status;
        if(intendedState == true){
            status = "Active";}
        else{
            status = "not Active";}
        if (elevatorSubsystem.m_proxSwitchBottomStateNew == intendedState){
            System.out.println("PASS: Elevator Prox Switch is " + status);
        }
        else{
            System.out.println("FAIL: Elevator Prox Switch is " + status);
            m_allTestsPassed = false;
        }
    }

    public void testElevatorEncoder(boolean intendedZero){
        String zero;
        if(intendedZero == true){
            zero = "at zero";}
        else{
            zero = "not at zero";}
        if(intendedZero == true){
            if (elevatorSubsystem.getPosition() == 0){
                System.out.println("PASS: Elevator Encoder is " + zero);
            }
            else{
                System.out.println("FAIL: Elevator Encoder is " + zero);
                m_allTestsPassed = false;
            }
        }
        else{
            if (elevatorSubsystem.getPosition() == 0){
                System.out.println("FAIL: Elevator Encoder is " + zero);
            }
            else{
                System.out.println("PASS: Elevator Encoder is " + zero);
                m_allTestsPassed = false;
            }
        }
    }

    public void testClawEncoder(boolean intendedStartSpot){
        if(intendedStartSpot == true){
            if (clawSubsystem.getPosition() < ClawConstants.CLAW_ELEVATOR_CLEAR + 0.5 && 
                clawSubsystem.getPosition() > ClawConstants.CLAW_ELEVATOR_CLEAR - 0.5 ){
                System.out.println("PASS: Shooter Pivot Encoder is at start spot");
            }
            else{
                System.out.println("FAIL: Shooter Pivot Encoder is not at start spot");
                m_allTestsPassed = false;
            }
        }
        else{
            if (clawSubsystem.getPosition() > ClawConstants.CLAW_ELEVATOR_CLEAR + 0.5){
                System.out.println("PASS: Shooter Pivot Encoder has moved from start spot");
            }
            else{
                System.out.println("FAIL: Shooter Pivot Encoder has not moved from start spot");
                m_allTestsPassed = false;
            }
        }
    }
        
    public void testIntakeRollers(){
        if(intakeSubsystem.getRollerSpeed() > 0){
            System.out.println("PASS: Intake Wheels rotated");}
        else{
            System.out.println("FAIL: Intake Wheels did not rotate");
            m_allTestsPassed = false;
        }
    }

    public void testResult(){
        if(m_allTestsPassed == true ){
            System.out.println("PASS: Passed all tests");}
        else{
            System.out.println("FAIL: Some tests failed");
        }
    }


    public Command getTestCommand(){
        return Commands.runOnce( ()-> testElevatorBeamBreak(true))
        .andThen(
            Commands.runOnce( ()-> testElevatorEncoder(true)),
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0.25), elevatorSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testElevatorEncoder(false)),
            Commands.runOnce( ()-> testElevatorBeamBreak(false)),
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(-0.25), elevatorSubsystem),
            Commands.waitSeconds(0.3),
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0), elevatorSubsystem),
            Commands.runOnce( ()-> testClawEncoder(true)),
            Commands.runOnce( ()-> clawSubsystem.setClawJog(0.25), clawSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testClawEncoder(false)),
            Commands.runOnce( ()-> clawSubsystem.setClawJog(-0.25), clawSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> clawSubsystem.setClawJog(0), clawSubsystem),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller(0.1)),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testIntakeRollers()),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller(0.0)),
            Commands.runOnce( ()-> testResult())
        );
    }
}
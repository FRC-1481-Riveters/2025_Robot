package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class RobotContainer 
{
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem( this );
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem( elevatorSubsystem );
    private final ShooterPivotSubsystem shooterPivotSubsystem = new ShooterPivotSubsystem();

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
    
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> getDriverMoveFwdBack(),
                () -> getDriverMoveLeftRight(),
                () -> getDriverRotate(),
                () -> !driverJoystick.getHID().getRightBumper() ));

        configureButtonBindings();

        // Register named pathplanner commands
        NamedCommands.registerCommand("ShootCommand", AutonShooterCommand());
        NamedCommands.registerCommand("ShootAgainCommand", AutonShootAgainCommand());
        NamedCommands.registerCommand("Shoot3FootCommand", AutonShooter3FootCommand());
        NamedCommands.registerCommand("SpewCommand", AutonSpewCommand());
        NamedCommands.registerCommand("IntakeRetractCommand", IntakeRetractAutoCommand() );
        NamedCommands.registerCommand("IntakeDeployCommand", IntakeDeployAutoCommand() );
        NamedCommands.registerCommand("IntakeRollersIn", IntakeRollersInCommand() );
        NamedCommands.registerCommand("IntakeRollersStop", IntakeRollersStopCommand() );
        NamedCommands.registerCommand("Stow", AutonStowCommand());
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
        Trigger aButton = driverJoystick.start();
        aButton
            .onTrue( Commands.runOnce( () -> swerveSubsystem.zeroHeading(180.0) ) );

        Trigger driverLeftTrigger = driverJoystick.leftTrigger( 0.7 );
        driverLeftTrigger
            .onFalse(Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_NORMAL )))
            .onTrue( Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_TURBO )));
        
        Trigger driverShootTrigger = driverJoystick.a();
        driverShootTrigger
            .onFalse(
                Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem )
                    .andThen( Commands.runOnce( ()->setBling(0, 0, 0) ) )
            )
            .onTrue( 
                ShooterCommand()
            );

        Trigger driverShoot3FootTrigger = driverJoystick.y();
        driverShoot3FootTrigger
            .onFalse(
                Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem )
                    .andThen( Commands.runOnce( ()->setBling(0, 0, 0) ) )
            )
            .onTrue( 
                Shooter3FootCommand()
            );

        Trigger driverShootPodiumTrigger = driverJoystick.b();
        driverShootPodiumTrigger
            .onFalse(
                Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem )
                    .andThen( Commands.runOnce( ()->setBling(0, 0, 0) ) )
            )
            .onTrue( 
                ShooterPodiumCommand()
            );

        Trigger driverDPadLeft = driverJoystick.povLeft();
        driverDPadLeft
            .onTrue(Commands.runOnce( ()->System.out.println("IntakeHalf") )
                .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_HALF ), intakeSubsystem))
                 );

        Trigger operatorIntakeDeployTrigger = operatorJoystick.y();
        operatorIntakeDeployTrigger
            .onTrue( IntakeDeployCommand() );

    
        Trigger operatorIntakeRetractTrigger = operatorJoystick.a();
        operatorIntakeRetractTrigger
            .onTrue(IntakeRetractCommand());

        Trigger operatorIntakeWheelsInTrigger = operatorJoystick.leftBumper();
        operatorIntakeWheelsInTrigger
            .whileTrue( IntakeRollersInCommand())
            .onFalse( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ) ) );

        Trigger operatorIntakeWheelsOutTrigger = operatorJoystick.rightBumper();
        operatorIntakeWheelsOutTrigger
            .onFalse(IntakeRollersStopCommand())
            .onTrue( IntakeRollersOutCommand());


/*
        Trigger operatorLeftAxisLeft = operatorJoystick.axisLessThan(0, -0.15);
        operatorLeftAxisLeft
            // intake cam
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setCamJog( 0 ), intakeSubsystem))
            .whileTrue( Commands.run( ()-> intakeSubsystem.setCamJog( operatorJoystick.getRawAxis(0) ), intakeSubsystem));

        Trigger operatorLeftAxisRight = operatorJoystick.axisGreaterThan(0, 0.15);
        operatorLeftAxisRight
            // intake cam
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setCamJog( 0 ), intakeSubsystem))
            .whileTrue( Commands.run( ()-> intakeSubsystem.setCamJog( operatorJoystick.getRawAxis(0) ), intakeSubsystem));
*/

        Trigger operatorAndDriverClimbTrigger = operatorJoystick.leftTrigger( 0.15 ) .and(driverJoystick.rightTrigger(0.15));
        operatorAndDriverClimbTrigger
            // spool climb
            .onFalse(Commands.runOnce( ()-> climbSubsystem.setClimbJog( 0 ), climbSubsystem))
            .whileTrue( 
                Commands.runOnce( ()->System.out.println("Climb Sequence") ) 
                .andThen( 
                    Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem)
                )
                .andThen( Commands.waitSeconds(150)
                    .until( shooterPivotSubsystem::atSetpoint)
                )
                .andThen( Commands.run( ()-> climbSubsystem.setClimbJog( -operatorJoystick.getLeftTriggerAxis() ), climbSubsystem) )
            );

        Trigger operatorRightTrigger = operatorJoystick.rightTrigger( 0.15 );
        operatorRightTrigger
            // zero elevator
            .onTrue( Commands.run( ()-> elevatorSubsystem.zeroEncoder(), elevatorSubsystem));
   
        Trigger operatorRightJoystickAxisUp = operatorJoystick.axisGreaterThan(5, 0.7 );
        operatorRightJoystickAxisUp
            .onFalse(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 0 ), shooterPivotSubsystem))
            .onTrue( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( -1.0 ), shooterPivotSubsystem));
        
        Trigger operatorRightJoystickAxisDown = operatorJoystick.axisLessThan(5, -0.7 );
        operatorRightJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 0 ), shooterPivotSubsystem))
            .onTrue( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 1.0 ), shooterPivotSubsystem));
        
        Trigger operatorLeftJoystickAxisUp = operatorJoystick.axisGreaterThan(1, 0.7 );
        operatorLeftJoystickAxisUp 
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), elevatorSubsystem))
            .onTrue( Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0.35 ), elevatorSubsystem));
        
        Trigger operatorLeftJoystickAxisDown = operatorJoystick.axisLessThan(1, -0.7 );
        operatorLeftJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), elevatorSubsystem))
            .onTrue(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem)
                .andThen( Commands.waitSeconds(150)
                    .until( shooterPivotSubsystem::atSetpoint)
                )
                .andThen(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( -0.20 ), elevatorSubsystem)));

        Trigger operatorRightJoystickAxisLeft = operatorJoystick.axisLessThan(4, -0.7 );
        operatorRightJoystickAxisLeft
            .onFalse( Commands.runOnce( ()->shooterSubsystem.setShooterJog(0), shooterSubsystem))
            .onTrue( Commands.runOnce( ()->shooterSubsystem.setShooterJog(-1), shooterSubsystem));
        
        Trigger operatorRightJoystickAxisRight = operatorJoystick.axisGreaterThan(4, 0.7 );
        operatorRightJoystickAxisRight
            .onFalse( Commands.runOnce( ()->shooterSubsystem.setShooterJog(0), shooterSubsystem))
            .onTrue( Commands.runOnce( ()->shooterSubsystem.setShooterJog(1), shooterSubsystem));
        
        //Close
        Trigger operatorDPadLeft = operatorJoystick.povLeft();
        operatorDPadLeft
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
        )
        .whileTrue(
            Commands.runOnce( ()->System.out.println("Close Operator Sequence") )      
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLOSE), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen(  Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem) )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_SPEAKER), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLOSE), shooterPivotSubsystem)
            )
        );
            

        //3foot
        Trigger operatorDPadUp = operatorJoystick.povUp();
        operatorDPadUp
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
        )      
        .whileTrue(
            Commands.runOnce( ()->System.out.println("3Foot Operator Sequence") ) 
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_3FOOT), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen(  Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem) )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_3FOOT), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_3FOOT), shooterPivotSubsystem)
            )
        );
        
        //podium
        Trigger operatorDPadRight = operatorJoystick.povRight();
        operatorDPadRight
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
        )      
        .whileTrue(
            Commands.runOnce( ()->System.out.println("Podium Operator Sequence") ) 
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PODIUM), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen(  Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem) )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_PODIUM), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_PODIUM), shooterPivotSubsystem)
            )
        );
        
/*
        Trigger operatorXTrigger = operatorJoystick.x();
        operatorXTrigger
         .onFalse(
                Commands.runOnce( ()-> intakeSubsystem.setCamJog(0), intakeSubsystem)
                  )
        .whileTrue(
            Commands.runOnce( ()->System.out.println("CAM Close Sequence") )
            .andThen( 
                         Commands.runOnce( ()-> intakeSubsystem.setCamPosition(IntakeConstants.INTAKE_CAM_SPEAKER), intakeSubsystem)
            )
        );

        Trigger operatorBTrigger = operatorJoystick.b();
        operatorBTrigger
         .onFalse(
                Commands.runOnce( ()-> intakeSubsystem.setCamJog(0), intakeSubsystem)
                  )
        .whileTrue(
            Commands.runOnce( ()->System.out.println("CAM 3foot Sequence") )
            .andThen( 
                         Commands.runOnce( ()-> intakeSubsystem.setCamPosition(IntakeConstants.INTAKE_CAM_3FOOT), intakeSubsystem)
            )
        );
*/
        //Amp OPERATOR HALF
        Trigger operatorDPadDown = operatorJoystick.povDown();
        operatorDPadDown
        .whileTrue(
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PIVOT_CLEAR), elevatorSubsystem)
            .andThen( Commands.waitSeconds(10)
                .until( elevatorSubsystem::isAtPosition))
            .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_AMP_LOAD), shooterPivotSubsystem))
            .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_AMP_LOAD), shooterSubsystem))
            .andThen( Commands.waitSeconds(3) 
                .until( shooterPivotSubsystem::atSetpoint))
            .andThen( Commands.waitSeconds(3)
                .until( intakeSubsystem::isAngleStable))
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0.7 ), intakeSubsystem))
            .andThen( Commands.waitSeconds(0.27))
              //  .until( shooterSubsystem::isLightCurtainBlocked))
            .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem))
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_AMP), elevatorSubsystem) )
            .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_AMP), shooterPivotSubsystem))
        );


        //amp DRIVER HALF
        //Don't allow driver amp shot if operator amp shot still in progress
        Trigger driverXTrigger = driverJoystick.x() .and(operatorDPadDown.negate());
        driverXTrigger
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
         )
        .whileTrue( Commands.runOnce( ()->System.out.println("AMP Driver Sequence") )
            .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_AMP), shooterSubsystem))
            .andThen( Commands.waitSeconds(1.75))
            .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem))
        );


        //Stow
        Trigger operatorBack = operatorJoystick.back();
        operatorBack
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterJog(0), shooterSubsystem)
            .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem) )
            .alongWith (
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0)),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0))
                )
            )      
        .whileTrue(
            Commands.runOnce( ()->System.out.println("Stow Sequence") ) 
            .andThen( 
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem)
            )
            .andThen( Commands.waitSeconds(3)
                .until( shooterPivotSubsystem::atSetpoint)
            )
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PAST_BUMP ), elevatorSubsystem)
            .andThen( Commands.waitSeconds(3)
                .until( elevatorSubsystem::isAtPosition)
            )
            .andThen( 
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLEAR_INTAKE), shooterPivotSubsystem)
            )
            .andThen( Commands.waitSeconds(3)
                .until( shooterPivotSubsystem::atSetpoint)
            )
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_START ), elevatorSubsystem)
            )
            .andThen( Commands.waitSeconds(10) 
                .until( elevatorSubsystem::isAtPosition ))
            )
            .andThen(
                Commands.runOnce( ()->setBling(0, 255, 0) ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.waitSeconds(0.5 ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->StopControls(true) )
            )
        );

        //climb start
        Trigger operatorStart = operatorJoystick.start();
        operatorStart  
         .onFalse(
            Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0))
            .alongWith (
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0))
                )
            )      
        .onTrue(
            Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLIMB_START ), elevatorSubsystem)
            .andThen( Commands.waitSeconds(10) 
                .until( elevatorSubsystem::isAtPosition ))
            .andThen(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLIMB), shooterPivotSubsystem))
            .andThen( Commands.waitSeconds(3)
                .until( shooterPivotSubsystem::atSetpoint)
            .andThen(
                Commands.runOnce( ()->setBling(0, 255, 0) ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 1) ),
                Commands.waitSeconds(0.5 ),
                Commands.runOnce( ()->driverJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ),
                Commands.runOnce( ()->operatorJoystick.getHID().setRumble(RumbleType.kBothRumble, 0) ) )
            )
        );
    }
    

    public Command ShooterCommand() 
    {
        return Commands.runOnce( ()->System.out.println("ShooterCommand") )
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_SPEAKER), shooterSubsystem) )
                .alongWith(
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLOSE), elevatorSubsystem)
                )
                .andThen( Commands.waitSeconds(3.0)
                    .until( elevatorSubsystem::isAtPosition ))
                .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLOSE), shooterPivotSubsystem) )
                .andThen( Commands.runOnce( ()->setBling(255, 255, 0)))
                .andThen( Commands.waitSeconds(3.0)
                    .until( this::isAtAllPositions ))
                .andThen( Commands.waitSeconds(3)
                    .until( intakeSubsystem::isAngleStable))
                .andThen( Commands.runOnce( ()->setBling(255, 0, 0)))
                .andThen(IntakeRollersOutCommand())
                .andThen(Commands.waitSeconds(0.60))
                .andThen( Commands.runOnce( ()->setBling(0, 0, 0)))
                .andThen(IntakeRollersStopCommand())
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0)) )
            ;
    }

    public Command Shooter3FootCommand() 
    {
        return Commands.runOnce( ()->System.out.println("Shooter3FootCommand") )
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_3FOOT), shooterSubsystem) )
                .alongWith(
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_3FOOT), elevatorSubsystem)
                )
                .andThen( Commands.waitSeconds(3.0)
                    .until( elevatorSubsystem::isAtPosition ))
                .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_3FOOT), shooterPivotSubsystem) )
                .andThen( Commands.runOnce( ()->setBling(255, 255, 0)))
                .andThen( Commands.waitSeconds(3.0)
                    .until( this::isAtAllPositions ))
                .andThen( Commands.waitSeconds(3)
                    .until( intakeSubsystem::isAngleStable))
                .andThen( Commands.runOnce( ()->setBling(255, 0, 0)))
                .andThen(IntakeRollersOutCommand())
                .andThen(Commands.waitSeconds(0.60))
                .andThen( Commands.runOnce( ()->setBling(0, 0, 0)))
                .andThen(IntakeRollersStopCommand())
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0)) )
            ;
    }

    public Command ShooterPodiumCommand() 
    {
        return Commands.runOnce( ()->System.out.println("ShooterPodiumCommand") )
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_PODIUM), shooterSubsystem) )
                .alongWith(
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PODIUM), elevatorSubsystem)
                )
                .andThen( Commands.waitSeconds(3.0)
                    .until( elevatorSubsystem::isAtPosition ))
                .andThen( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_PODIUM), shooterPivotSubsystem) )
                .andThen( Commands.runOnce( ()->setBling(255, 255, 0)))
                .andThen( Commands.waitSeconds(3.0)
                    .until( this::isAtAllPositions ))
                .andThen( Commands.waitSeconds(3)
                    .until( intakeSubsystem::isAngleStable))
                .andThen( Commands.runOnce( ()->setBling(255, 0, 0)))
                .andThen(IntakeRollersOutCommand())
                .andThen(Commands.waitSeconds(0.60))
                .andThen( Commands.runOnce( ()->setBling(0, 0, 0)))
                .andThen(IntakeRollersStopCommand())
                .andThen( Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0)) )
            ;
    }

    public Command AutonShooterCommand() 
    {
        return Commands.runOnce( ()->System.out.println("AutonShooterCommand") )
            .andThen(Commands.runOnce(()-> swerveSubsystem.saveOdometry()))
            /*.andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PIVOT_CLEAR), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            ) */
            .andThen( 
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLOSE), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen(  Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem) )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_SPEAKER), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLOSE), shooterPivotSubsystem)
            )
            .andThen( Commands.waitSeconds(3.0)
                .until( this::isAtAllPositions ))
            .andThen( IntakeRollersOutCommand())
            .andThen( Commands.waitSeconds(0.60))
            .andThen( IntakeRollersStopCommand())
            /* .andThen( 
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_3FOOT), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_3FOOT), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_3FOOT), elevatorSubsystem)
            ) */
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
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_CLOSE), elevatorSubsystem)
            )
            .andThen( 
                Commands.waitSeconds(3)
                    .until( elevatorSubsystem::isAboveIntake )
            )
            .andThen(  Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem) )
            .andThen( 
                Commands.waitSeconds(10)
                    .until( elevatorSubsystem::isAtPosition)
            )
            .andThen(   
                Commands.runOnce(()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_AUTON_SPEW), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLOSE), shooterPivotSubsystem)
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
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_3FOOT), shooterSubsystem),
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_3FOOT), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_3FOOT), elevatorSubsystem)
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
            .andThen(Commands.runOnce(()-> shooterSubsystem.setShooterJog(0)))
            .andThen( 
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL), shooterPivotSubsystem)
            )
            .andThen( Commands.waitSeconds(3)
                .until( shooterPivotSubsystem::atSetpoint)
            )
            .andThen( Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PAST_BUMP ), elevatorSubsystem)
            .andThen( Commands.waitSeconds(3)
                .until( elevatorSubsystem::isAtPosition)
            )
            .andThen( 
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_CLEAR_INTAKE), shooterPivotSubsystem)
            )
            .andThen( Commands.waitSeconds(3)
                .until( shooterPivotSubsystem::atSetpoint)
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
        return Commands.runOnce( ()->System.out.println("IntakeRetractCommand") )
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_ANGLE_STOWED ), intakeSubsystem))
            .andThen(Commands.waitSeconds(3))
                .until(intakeSubsystem::atSetpoint)
                .finallyDo( intakeSubsystem::intakeAngleDisable);
    }

    public Command IntakeDeployCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeDeployCommand") )
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_FLOOR_PICKUP ), intakeSubsystem))
            .andThen(Commands.waitSeconds(3))
                .until(intakeSubsystem::atSetpoint)
                .finallyDo( intakeSubsystem::intakeAngleDisable);
    }

    public Command IntakeRetractAutoCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRetractAutoCommand") )
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_ANGLE_STOWED ), intakeSubsystem));
    }

    public Command IntakeDeployAutoCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeDeployAutoCommand") )
            .andThen(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_FLOOR_PICKUP ), intakeSubsystem));
    }

    public Command IntakeRollersInCommand() 
    {
        return Commands.runOnce( ()->System.out.println("IntakeRollersInCommand") )
            .andThen( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( -0.7 ) ) )
            .andThen( Commands.waitSeconds(20)
                .until(intakeSubsystem::isIntakeBeamBreakLoaded))
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
        shooterPivotSubsystem.setShooterPivotJog(0);
        elevatorSubsystem.setElevatorJog(0);
        intakeSubsystem.setIntakeRoller( 0.0 );
        intakeSubsystem.setCamJog(0);
        shooterSubsystem.setShooterJog(0);
    }

    public boolean isAtAllPositions()
    {
        if( elevatorSubsystem.isAtPosition() && 
            shooterSubsystem.isAtSpeed() &&
            shooterPivotSubsystem.atSetpoint() )
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
        if (elevatorSubsystem.m_proxSwitchBottomState == intendedState){
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

    public void testShooterEncoder(){
        if(shooterSubsystem.getSpeed() > 0){
            System.out.println("PASS: Shooter Wheels rotated");}
        else{
            System.out.println("FAIL: Shooter Wheels did not rotate");}
            m_allTestsPassed = false;
    }

    public void testShooterPivotEncoder(boolean intendedStartSpot){
        if(intendedStartSpot == true){
            if (shooterPivotSubsystem.getPosition() < ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL + 0.5 && 
                shooterPivotSubsystem.getPosition() > ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL - 0.5 ){
                System.out.println("PASS: Shooter Pivot Encoder is at start spot");
            }
            else{
                System.out.println("FAIL: Shooter Pivot Encoder is not at start spot");
                m_allTestsPassed = false;
            }
        }
        else{
            if (shooterPivotSubsystem.getPosition() > ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL + 0.5){
                System.out.println("PASS: Shooter Pivot Encoder has moved from start spot");
            }
            else{
                System.out.println("FAIL: Shooter Pivot Encoder has not moved from start spot");
                m_allTestsPassed = false;
            }
        }
    }

    public void testIntakeAngle(boolean intendedStartSpot){
        if(intendedStartSpot == true){
            if (intakeSubsystem.getIntakeAngle() < IntakeConstants.INTAKE_ANGLE_STOWED + 10){
                System.out.println("PASS: Intake is at start spot");
            }
            else{
                System.out.println("FAIL: Intake is not at start spot");
                m_allTestsPassed = false;
            }
        }
        else{
            if (intakeSubsystem.getIntakeAngle() > ShooterPivotConstants.SHOOTER_PIVOT_TRAVEL + 10){
                System.out.println("PASS: Intake has moved from start spot");
            }
            else{
                System.out.println("FAIL: Intake has not moved from start spot");
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
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(100), shooterSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testShooterEncoder()),
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem),
            Commands.runOnce( ()-> testShooterPivotEncoder(true)),
            Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0.25), shooterPivotSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testShooterPivotEncoder(false)),
            Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(-0.25), shooterPivotSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem),
            Commands.runOnce( ()-> testIntakeAngle(true)),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_SOURCE), intakeSubsystem),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testIntakeAngle(false)),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_ANGLE_STOWED), intakeSubsystem),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller(0.1)),
            Commands.waitSeconds(0.25),
            Commands.runOnce( ()-> testIntakeRollers()),
            Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller(0.0)),
            Commands.runOnce( ()-> testResult())
        );
    }
 }

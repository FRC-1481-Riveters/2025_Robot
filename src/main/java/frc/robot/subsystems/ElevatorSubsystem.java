package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_elevatorMotor =  new TalonFX(ElevatorConstants.ELEVATOR_MOTOR, "rio");
    private final TalonFX m_elevatorMotorFollower =  new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_FOLLOWER, "rio");
    private final Follower follower = new Follower(m_elevatorMotor.getDeviceID(), false);
    //private final boolean m_CANCoderReversed;
    //private final double m_CANCoderOffsetDegrees;
    
    private final PIDController elevatorPidController = new PIDController(ElevatorConstants.ELEVATOR_MOTOR_BELOW_KP, ElevatorConstants.ELEVATOR_MOTOR_BELOW_KI, ElevatorConstants.ELEVATOR_MOTOR_BELOW_KD);

    public static final double elevator_kA = 0.12872;
    public static final double elevator_kV = 2.3014;
    public static final double elevator_kS = 0.55493;
    InvertedValue elevatorMotorInverted = InvertedValue.Clockwise_Positive; 
    InvertedValue elevatorFollowerMotorInverted = InvertedValue.Clockwise_Positive;

    /*private SparkMax m_motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless );
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(ElevatorConstants.ELEVATOR_VELOCITY, ElevatorConstants.ELEVATOR_ACCELERATION);
    private ProfiledPIDController pidElevator = new ProfiledPIDController(0.30, 0.090, 0.005, m_constraints, 0.02);*/
    private DigitalInput m_proxSwitchBottom = new DigitalInput(0);
    public boolean m_proxSwitchBottomStatePrevious;
    public boolean m_proxSwitchBottomStateNew;


    private boolean m_pid;
    private double m_setpoint;
    private double m_position;
    private boolean m_atPosition;

    public ElevatorSubsystem(){

      MotorOutputConfigs elevatorMotorOutputConfigs = new MotorOutputConfigs();
      CurrentLimitsConfigs elevatorMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
      elevatorPidController.setIZone(0.05);
      
    elevatorMotorOutputConfigs
        .withNeutralMode(NeutralModeValue.Brake);
    elevatorMotorCurrentLimitsConfigs
        .withSupplyCurrentLimit(15)
        .withSupplyCurrentLimitEnable(true);
        //.withStatorCurrentLimit(180)
        //.withStatorCurrentLimitEnable(true);
    // elevatorMotor.enableVoltageCompensation(true);
    m_elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_elevatorMotor.getConfigurator().apply(elevatorMotorCurrentLimitsConfigs);
    m_elevatorMotor.getConfigurator().apply(elevatorMotorOutputConfigs);
    m_elevatorMotor.setPosition(0);
  
    // elevatorMotor.enableVoltageCompensation(true);
    m_elevatorMotorFollower.getConfigurator().apply(new TalonFXConfiguration());
    m_elevatorMotorFollower.getConfigurator().apply(elevatorMotorCurrentLimitsConfigs);
    m_elevatorMotorFollower.getConfigurator().apply(elevatorMotorOutputConfigs);
    m_elevatorMotorFollower.setPosition(0);

    m_elevatorMotorFollower.setControl(follower);

      // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
      Logger.recordOutput("Elevator/Position", 0.0 );
      Logger.recordOutput("Elevator/Setpoint", 0.0);
      Logger.recordOutput("Elevator/Output", 0.0 );
      Logger.recordOutput("Elevator/BeamBreak", false );
      Logger.recordOutput("Elevator/AtPosition", false );

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double pidCalculate;
      double output;
      m_position =  m_elevatorMotor.getPosition().getValueAsDouble();

      // This method will be called once per scheduler run
      m_proxSwitchBottomStateNew = !m_proxSwitchBottom.get();
      Logger.recordOutput("Elevator/ProxSwitchBottom", m_proxSwitchBottomStateNew );

      m_atPosition = false;
      
      if(  m_proxSwitchBottomStatePrevious == false && m_proxSwitchBottomStateNew == true)
      {
        m_position = 1;
        m_elevatorMotor.setPosition(m_position);
      }
      m_proxSwitchBottomStatePrevious = m_proxSwitchBottomStateNew;

      if( m_pid == true )
      {
        pidCalculate = elevatorPidController.calculate( m_position, m_setpoint);
        output = MathUtil.clamp( pidCalculate, -0.75, 0.75);
        m_elevatorMotor.set( output );
        Logger.recordOutput("Elevator/Output", output);
        Logger.recordOutput("Elevator/Current", m_elevatorMotor.getStatorCurrent().getValueAsDouble());
        if (Math.abs((m_position - m_setpoint)) <= ElevatorConstants.ELEVATOR_POSITION_TOLERANCE)
        {
           m_atPosition = true;
        }
      }

      Logger.recordOutput("Elevator/Position", m_position );
      Logger.recordOutput("Elevator/AtPosition", m_atPosition );
    } // end of method

    public void setElevatorJog( double speed )
    {
      m_pid = false;
      m_elevatorMotor.set(speed);
      Logger.recordOutput("Elevator/Output", speed);
      System.out.println("setElevatorJog " + speed );
    }

    public void setElevatorPosition (double position){
        System.out.println("setElevatorPosition " + position);

        m_setpoint = position;    
        m_pid = true;
      if( m_position > m_setpoint )
      {
        elevatorPidController.setP( ElevatorConstants.ELEVATOR_MOTOR_ABOVE_KP );
        elevatorPidController.setI( ElevatorConstants.ELEVATOR_MOTOR_ABOVE_KI );
        elevatorPidController.setD( ElevatorConstants.ELEVATOR_MOTOR_ABOVE_KD );
      }
      else if( m_position < m_setpoint  )
      {
      elevatorPidController.setP( ElevatorConstants.ELEVATOR_MOTOR_BELOW_KP );
      elevatorPidController.setI( ElevatorConstants.ELEVATOR_MOTOR_BELOW_KI );
      elevatorPidController.setD( ElevatorConstants.ELEVATOR_MOTOR_BELOW_KD );
      }
        elevatorPidController.setSetpoint(position);
        Logger.recordOutput("Elevator/Setpoint", m_setpoint);
    }

    public double getPosition() {
        return m_position;
    }
    
    public boolean isAboveIntake()
    {
      boolean retval;
      if( m_position < ElevatorConstants.ELEVATOR_BARGE )
        retval = true;
      else
        retval = false;
      return retval;
    }

    public boolean isAtPosition() 
    {
      return m_atPosition;
    }

    public void elevatorDisable()
    {
        m_elevatorMotor.set(0.0);
        m_elevatorMotorFollower.set(0.0);
        m_pid = false;
        System.out.println("elevatorDisable current position=" + getPosition());
        Logger.recordOutput("Elevator/Output", 0.0);
    }

    public void zeroEncoder()
    {
        m_position = 1;
        m_elevatorMotor.setPosition(m_position);
        m_elevatorMotorFollower.setPosition(m_position);
    }
}

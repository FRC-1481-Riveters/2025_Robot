package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterPivotConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax m_motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless );
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(ElevatorConstants.ELEVATOR_VELOCITY, ElevatorConstants.ELEVATOR_ACCELERATION);
    private ProfiledPIDController pidElevator = new ProfiledPIDController(0.30, 0.090, 0.005, m_constraints, 0.02);
    private DigitalInput m_proxSwitchBottom = new DigitalInput(3);
    public boolean m_proxSwitchBottomState;

    private boolean m_pid;
    private double m_setpoint;
    private double m_position;
    private boolean m_atPosition;

    public ElevatorSubsystem(){
      //m_motor.restoreFactoryDefaults();
      m_motorConfig
        .inverted(true)
        .secondaryCurrentLimit(m_position, 0)
        .idleMode(IdleMode.kBrake)
        .softLimit
        .reverseSoftLimit( (float) ElevatorConstants.ELEVATOR_MAX)
        .reverseSoftLimitEnabled(true);
        
      m_motorConfig.smartCurrentLimit(30, 30);
      m_encoder.setPosition(0);

      // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
      Logger.recordOutput("Elevator/Position", 0.0 );
      Logger.recordOutput("Elevator/Setpoint", 0.0);
      Logger.recordOutput("Elevator/Output", 0.0 );
      Logger.recordOutput("Elevator/BeamBreak", false );
      Logger.recordOutput("Elevator/AtPosition", false );

      pidElevator.setIZone(1.0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double pidCalculate;
      double output;
      m_position = m_encoder.getPosition();

      // This method will be called once per scheduler run
      m_proxSwitchBottomState = !m_proxSwitchBottom.get();
      Logger.recordOutput("Elevator/ProxSwitchBottom", m_proxSwitchBottomState );

      // If the elevator is all the way down, zero the encoder
      if( m_proxSwitchBottomState == true )
      {
        m_position = 0;
        m_encoder.setPosition(m_position);
      }

      m_atPosition = false;

      if( m_pid == true )
      {
        pidCalculate = pidElevator.calculate( m_position, m_setpoint);
        output = MathUtil.clamp( pidCalculate, -0.75, 0.75);
        m_motor.set( output );
        Logger.recordOutput("Elevator/Output", output);
        Logger.recordOutput("Elevator/Current", m_motor.getOutputCurrent());
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
      m_motor.set(speed);
      Logger.recordOutput("Elevator/Output", speed);
      System.out.println("setElevatorJog " + speed );
    }

    public void setElevatorPosition (double position){
        System.out.println("setElevatorPosition " + position);

        m_setpoint = position;    
        m_pid = true;
        pidElevator.reset(m_position);
        Logger.recordOutput("Elevator/Setpoint", m_setpoint);
    }

    public double getPosition() {
        return m_position;
    }
    
    public boolean isAboveIntake()
    {
      boolean retval;
      if( m_position < ElevatorConstants.ELEVATOR_ABOVE_BUMP )
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
        m_motor.set(0.0);
        m_pid = false;
        System.out.println("elevatorDisable current position=" + getPosition());
        Logger.recordOutput("Elevator/Output", 0.0);
    }

    public void zeroEncoder()
    {
        m_position = 0;
        m_encoder.setPosition(m_position);
    }
}

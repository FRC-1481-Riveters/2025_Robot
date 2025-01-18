package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.sensors.CANCoder;

import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase 
{
    private SparkMax m_motor = new SparkMax(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR, SparkLowLevel.MotorType.kBrushless );
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private CANCoder m_CANCoder = new CANCoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(ShooterPivotConstants.SHOOTER_PIVOT_VELOCITY, ShooterPivotConstants.SHOOTER_PIVOT_ACCELERATION);
    private ProfiledPIDController pid = new ProfiledPIDController(
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KP,
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KI,
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KD,
                                        m_constraints, 0.02
                                    );
    private double m_Setpoint;
    private double m_output;
    private double m_position;
    private double m_tolerance;
    private boolean m_atSetpoint;
    private int m_atSetpointDebounceCounter;
    private boolean m_pid;


    public ShooterPivotSubsystem() 
    {
        m_CANCoder.setPosition(m_CANCoder.getAbsolutePosition());

        m_tolerance = 2.5;
        pid.setIZone(m_tolerance*3);

      //  m_motor.restoreFactoryDefaults();
        m_motorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30, 20);
        m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("ShooterPivot/Setpoint", 0.0 );
        Logger.recordOutput("ShooterPivot/Position", 0.0);
        Logger.recordOutput("ShooterPivot/Output", 0.0);
        Logger.recordOutput("ShooterPivot/AtSetpoint", false );
    }

    public void setShooterPivot( double angle )
    {
        double sensorSetpoint;

        m_Setpoint = angle;
        pid.setIZone(m_tolerance*3);
        if( (angle >= (ShooterPivotConstants.SHOOTER_PIVOT_CLOSE - 0.5)) &&
            (angle <= (ShooterPivotConstants.SHOOTER_PIVOT_CLOSE + 0.5)) )
        {
            pid.setP( ShooterPivotConstants.SHOOTER_PIVOT_CLOSE_KP );
            pid.setI( ShooterPivotConstants.SHOOTER_PIVOT_CLOSE_KI );
            pid.setD( ShooterPivotConstants.SHOOTER_PIVOT_CLOSE_KD );
            pid.setIZone(m_tolerance*4);
        }
        else if( (angle >= (ShooterPivotConstants.SHOOTER_PIVOT_AMP - 0.5)) &&
            (angle <= (ShooterPivotConstants.SHOOTER_PIVOT_AMP + 0.5)) )
        {
            pid.setP( ShooterPivotConstants.SHOOTER_PIVOT_AMP_KP );
            pid.setI( ShooterPivotConstants.SHOOTER_PIVOT_AMP_KI );
            pid.setD( ShooterPivotConstants.SHOOTER_PIVOT_AMP_KD );
        }
        else
        {
            pid.setP( ShooterPivotConstants.SHOOTER_PIVOT_0_KP );
            pid.setI( ShooterPivotConstants.SHOOTER_PIVOT_0_KI );
            pid.setD( ShooterPivotConstants.SHOOTER_PIVOT_0_KD );
        }
        pid.reset(m_position);
        m_pid = true;
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );

        System.out.println("setShooterPivot " + angle + ", current angle=" + m_position);
    }

    public void setShooterPivotJog( double speed )
    {
        m_pid = false;
        m_output = speed;
        m_Setpoint = 0;
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );
        System.out.println("setShooterPivotJog " + m_output );
    }

    public boolean atSetpoint()
    {        
        return m_atSetpoint;
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        double pidCalculate;
  
        m_position = m_CANCoder.getAbsolutePosition();
        if( m_position > 300 )
        {
            m_position = m_position - 360;
        }

        if( m_pid == true )
        {
          pidCalculate = pid.calculate( m_position, m_Setpoint);
          m_output = MathUtil.clamp( pidCalculate, -0.25, 0.25);
        }

        if( (m_position > ShooterPivotConstants.SHOOTER_PIVOT_MAX) ||
            (m_position < ShooterPivotConstants.SHOOTER_PIVOT_MIN) )
        {
            m_output = 0;
        }
    
        m_motor.set( m_output );
        Logger.recordOutput("ShooterPivot/Output", m_output);
        Logger.recordOutput("ShooterPivot/Current",m_motor.getOutputCurrent());

        Logger.recordOutput("ShooterPivot/Position", m_position);
        if( Math.abs( m_position - m_Setpoint ) > m_tolerance )
        {
            m_atSetpoint = false;
            m_atSetpointDebounceCounter = 0;
            Logger.recordOutput("ShooterPivot/AtSetpoint", m_atSetpoint );
        }
        else if( m_atSetpointDebounceCounter < 12 )
        {
            m_atSetpointDebounceCounter++;
            if( m_atSetpointDebounceCounter == 12 )
            {
                m_atSetpoint = true;
                Logger.recordOutput("ShooterPivot/AtSetpoint", m_atSetpoint );
            }
        }
    }

    public double getPosition(){
        return m_position;
    }

}

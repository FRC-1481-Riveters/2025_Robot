package frc.robot.subsystems;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;



import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
/* 
    private SparkMax m_motor = new SparkMax(ClimbConstants.CLIMB_MOTOR, SparkLowLevel.MotorType.kBrushless );
    private SparkMax m_motorFollower = new SparkMax(ClimbConstants.CLIMB_MOTOR_FOLLOWER, SparkLowLevel.MotorType.kBrushless );
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_motorFollowerConfig = new SparkMaxConfig();
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private double m_setpoint;
    private double m_position;
    private ElevatorSubsystem m_elevatorSubsystem;

    public ClimbSubsystem( ElevatorSubsystem elevatorSubsystem ) 
    {
        m_elevatorSubsystem = elevatorSubsystem;
        //m_motor.restoreFactoryDefaults();
        m_motorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        m_motorConfig.smartCurrentLimit(80, 50);
        m_encoder.setPosition(0);
        //m_motorFollower.restoreFactoryDefaults();
        m_motorFollowerConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80, 50);
//        m_motorFollower.follow(m_motor,false);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Climb/Position", 0.0 );
        Logger.recordOutput("Climb/Output", 0.0 );
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        m_position = m_encoder.getPosition();
        double elevator_position = m_elevatorSubsystem.getPosition();

        // This method will be called once per scheduler run
        Logger.recordOutput("Climb/Position", m_position );

        if( (m_setpoint < 0) &&
            ( 
              (elevator_position < (ElevatorConstants.ELEVATOR_CLIMB_FULL + 0.5) ) ||
              (m_position < ClimbConstants.CLIMB_ENCODER_FULLY_CLIMBED)
            )
          )
        {
            // climb complete - STOP SPOOLING
            m_setpoint = 0.0;
            m_motor.set(m_setpoint);
            m_motorFollower.set(m_setpoint);
            Logger.recordOutput("Climb/Output", m_setpoint );
            Logger.recordOutput("Climb/Current", m_motor.getOutputCurrent());
            Logger.recordOutput("Climb/CurrentFollower", m_motorFollower.getOutputCurrent());
        }
    }

    public void setClimbJog( double percentOutput )
    {
        double elevator_position = m_elevatorSubsystem.getPosition();

        System.out.println("setClimbJog: output=" + percentOutput + " elevator_position=" + elevator_position);
        if( percentOutput > 0 )
        {
            // unspooling - operator right trigger
            m_setpoint = percentOutput;
            m_motor.set(percentOutput);
            m_motorFollower.set(percentOutput);
        }
        else if( (percentOutput < 0) &&
            (elevator_position < (ElevatorConstants.ELEVATOR_CLIMB_FULL + 0.5) ) &&
              (m_position > ClimbConstants.CLIMB_ENCODER_FULLY_CLIMBED) )
        {
            // spooling - operator left trigger
            m_setpoint = percentOutput;
            m_motor.set(percentOutput);
            m_motorFollower.set(percentOutput);
        }
        else 
        {
            m_setpoint = 0.0;
            m_motor.set(m_setpoint);
            m_motorFollower.set(m_setpoint);
        }

        Logger.recordOutput("Climb/Output", m_setpoint );
    }*/
}


package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import org.littletonrobotics.junction.Logger;

public class ClawSubsystem extends SubsystemBase 
{
    private final TalonFX m_clawMotor =  new TalonFX(ClawConstants.CLAW_MOTOR, "rio");

    //private final boolean m_CANCoderReversed;
    //private final double m_CANCoderOffsetDegrees;
    
    private final PIDController clawPidController = new PIDController(ClawConstants.CLAW_KP, ClawConstants.CLAW_KI, ClawConstants.CLAW_KD);
    
    public static final double claw_kA = 0.12872;
    public static final double claw_kV = 2.3014;
    public static final double claw_kS = 0.55493;
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward( claw_kS, claw_kV, claw_kA ); InvertedValue clawMotorInverted = InvertedValue.CounterClockwise_Positive; //check direction for drive (is true the same as clockwise / counter-clockwise)
    
   
    private double m_Setpoint;
    private double m_output;
    private double m_position;
    private double m_tolerance;
    private boolean m_atSetpoint;
    private int m_atSetpointDebounceCounter;
    private boolean m_pid;


    public ClawSubsystem() 
    {
        InvertedValue clawMotorInverted =InvertedValue.CounterClockwise_Positive; //check direction for drive (is true the same as clockwise / counter-clockwise)

        m_clawMotor.setPosition(ClawConstants.CLAW_START);

        MotorOutputConfigs clawMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs clawMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration();
        currentConfig.currentLimit = 1;
        currentConfig.enable = true;

        clawMotorOutputConfigs
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(clawMotorInverted);
        clawMotorCurrentLimitsConfigs
            .withSupplyCurrentLimit(3)
            .withSupplyCurrentLimitEnable(true);
        // clawMotor.configVoltageCompSaturation(12.5);
        // clawMotor.enableVoltageCompensation(true);
        m_clawMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_clawMotor.getConfigurator().apply(clawMotorCurrentLimitsConfigs);
        m_clawMotor.getConfigurator().apply(clawMotorOutputConfigs);


       // m_CANCoder.setPosition(m_CANCoderOffsetDegrees);

        m_tolerance = 2.5;
        clawPidController.setIZone(m_tolerance*3);
        
        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Claw/Setpoint", 0.0 );
        Logger.recordOutput("Claw/Position", 0.0);
        Logger.recordOutput("Claw/Output", 0.0);
        Logger.recordOutput("Claw/AtSetpoint", false );
    }

    public void setClaw( double angle )
    {
        m_Setpoint = angle;
        clawPidController.reset();
        m_pid = true;
        Logger.recordOutput("Claw/Setpoint", m_Setpoint );

        System.out.println("setClaw " + angle + ", current angle=" + m_position);
    }

    public void setClawJog( double speed )
    {
        m_pid = false;
        m_output = speed;
        m_clawMotor.set(speed);
        m_Setpoint = 0;
        Logger.recordOutput("Claw/Setpoint", m_Setpoint );
        System.out.println("setClawJog " + m_output );
    }

    public boolean atSetpoint()
    {        
        return m_atSetpoint;
    }

    public boolean ClawPosition()
    {      
        if (m_clawMotor.getPosition().getValueAsDouble() < 4.0 ){ 
        return true;
        }
        else{
         return false;
        }
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        double pidCalculate;
        m_position = m_clawMotor.getPosition().getValueAsDouble();
        if( m_position > 300 )
        {
            m_position = m_position - 360;
        }

        if( m_pid == true )
        {
          pidCalculate = clawPidController.calculate( m_position, m_Setpoint);
          m_output = MathUtil.clamp( pidCalculate, -0.25, 0.25);
        }

        /*if( (m_position > ClawConstants.CLAW_MAX) ||
            (m_position < ClawConstants.CLAW_MIN) )
        {
            m_output = 0;
        }*/
    
        m_clawMotor.set( m_output );
        Logger.recordOutput("Claw/Output", m_output);
        Logger.recordOutput("Claw/Current",m_clawMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput("Claw/Position", m_position);
        if( Math.abs( m_position - m_Setpoint ) > m_tolerance )
        {
            m_atSetpoint = false;
            m_atSetpointDebounceCounter = 0;
            Logger.recordOutput("Claw/AtSetpoint", m_atSetpoint );
        }
        else if( m_atSetpointDebounceCounter < 12 )
        {
            m_atSetpointDebounceCounter++;
            if( m_atSetpointDebounceCounter == 12 )
            {
                m_atSetpoint = true;
                Logger.recordOutput("Claw/AtSetpoint", m_atSetpoint );
            }
        }
    }

    public double getPosition(){
        return m_position;
    }

}

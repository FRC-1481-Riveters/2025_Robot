package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.*;
//import com.revrobotics.SparkBase.ControlType;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase
{
    private double m_intendedSpeed;
    private SparkMax m_bottomMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM_FORWARD, SparkLowLevel.MotorType.kBrushless );
    private SparkMax m_bottomBackMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM_BACK, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_topMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_FORWARD, SparkLowLevel.MotorType.kBrushless );
    private SparkMax m_topBackMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_BACK, SparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_bottomEncoder = (SparkRelativeEncoder) m_bottomMotor.getEncoder();
    private SparkRelativeEncoder m_bottomBackEncoder = (SparkRelativeEncoder) m_bottomBackMotor.getEncoder();
    private SparkRelativeEncoder m_topEncoder = (SparkRelativeEncoder) m_topMotor.getEncoder();
    private SparkRelativeEncoder m_topBackEncoder = (SparkRelativeEncoder) m_topBackMotor.getEncoder();
    private SparkMaxConfig m_bottomMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_bottomBackMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_topMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_topBackMotorConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_bottomPid = m_bottomMotor.getClosedLoopController();
    private SparkClosedLoopController m_topPid = m_topMotor.getClosedLoopController();
    private SparkClosedLoopController m_bottomBackPid = m_bottomBackMotor.getClosedLoopController();
    private SparkClosedLoopController m_topBackPid = m_topBackMotor.getClosedLoopController();

    DigitalInput m_beamBreak = new DigitalInput(2);
    
    public ShooterSubsystem()
    {
    // m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotorConfig
         .inverted(true)
         .idleMode(IdleMode.kCoast);
        m_bottomMotorConfig.smartCurrentLimit(80, 50);
        m_bottomMotorConfig.closedLoop
        .pidf(0.00005, 0.000000060, 0.0001, 0.000145);
     //0.00018 FF

     // m_topMotor.restoreFactoryDefaults();
    m_topMotorConfig
        .inverted(true)
       // m_topMotor.setSmartCurrentLimit(80, 50);
        .idleMode(IdleMode.kCoast);
    m_topMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.00005, 0.000000060, 0.00001, 0.000145);
         //0.00018 FF

    //m_bottomBackMotor.restoreFactoryDefaults();
     m_bottomBackMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        m_bottomBackMotorConfig.smartCurrentLimit(80, 50);
        m_bottomBackMotorConfig.closedLoop
            .pidf(0.00005, 0.000000060, 0.0001,0.000145);
         //0.00018 FF

//        m_topBackMotor.restoreFactoryDefaults();
        m_topBackMotorConfig 
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        m_topBackMotorConfig.smartCurrentLimit(80, 50);
        m_topBackMotorConfig.closedLoop
            .pidf(0.00005, 0.000000060, 0.0001,0.000145);
            //0.00018 FF

        setShooterSpeed(0.0);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Shooter/Setpoint", 0.0 );
        Logger.recordOutput("Shooter/BottomSpeed", 0.0 );
        Logger.recordOutput("Shooter/BottomBackSpeed", 0.0 );
        Logger.recordOutput("Shooter/TopSpeed", 0.0 );
        Logger.recordOutput("Shooter/TopBackSpeed", 0.0 );
        Logger.recordOutput("Shooter/AtSpeed", false );
        Logger.recordOutput("Shooter/BeamBreak", false );
        Logger.recordOutput("Shooter/Jog", 0.0 );
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        Logger.recordOutput("Shooter/BottomSpeed", getSpeed() );
        Logger.recordOutput("Shooter/BottomBackSpeed", m_bottomBackEncoder.getVelocity() );
        Logger.recordOutput("Shooter/TopSpeed", m_topEncoder.getVelocity() );
        Logger.recordOutput("Shooter/TopBackSpeed", m_topBackEncoder.getVelocity() );
        Logger.recordOutput("Shooter/CurrentTopBack", m_topBackMotor.getOutputCurrent() );
        Logger.recordOutput("Shooter/CurrentTopForward", m_topMotor.getOutputCurrent() );
        Logger.recordOutput("Shooter/CurrentBottomBack", m_bottomBackMotor.getOutputCurrent() );
        Logger.recordOutput("Shooter/CurrentBottomForward", m_bottomMotor.getOutputCurrent() );

        Logger.recordOutput("Shooter/BeamBreak", isLightCurtainBlocked() );
    }

    public void setShooterSpeed (double rpm)
    {
        System.out.println("setShooterSpeed " + rpm);
        Logger.recordOutput("Shooter/Setpoint", rpm );
        Logger.recordOutput("Shooter/Jog", 0.0 );

        m_intendedSpeed = rpm;
        if (m_intendedSpeed > 10.0) 
        {
            // Spark MAX PID
<<<<<<< HEAD
            m_bottomPid.setReference(rpm, ControlType.kVelocity);
=======
            m_bottomPid.setReference(rpm, ControlType.kVelocity); 
>>>>>>> 32d85d9 (hello)
            m_topPid.setReference(rpm+50, ControlType.kVelocity);
            m_bottomBackPid.setReference(rpm, ControlType.kVelocity);
            m_topBackPid.setReference(rpm+50, ControlType.kVelocity);
        }
        else 
        {
            // Turn shooter off
            m_bottomMotor.set(0.0);
            m_topMotor.set(0.0);
            m_bottomBackMotor.set(0.0);
            m_topBackMotor.set(0.0);
        }
        
    }

    public void setShooterJog (double output){
        System.out.println("setShooterJog " + output);

        m_intendedSpeed = 0;
        m_bottomMotor.set(output);
        m_topMotor.set(output);
        m_bottomBackMotor.set(output);
        m_topBackMotor.set(output);
        Logger.recordOutput("Shooter/Setpoint", 0.0);
        Logger.recordOutput("Shooter/Jog", output );
   }

    public double getSpeed() {
        return m_bottomEncoder.getVelocity();
    }
    
    public boolean isAtSpeed() 
    {
        boolean retval;

        if( m_intendedSpeed == 0.0 )
            retval = true;
        else if (Math.abs(
            (getSpeed() - m_intendedSpeed) / m_intendedSpeed) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) 
        {
            // return TRUE if the shooter is at the right speed
            retval = true;
        }
        else 
        {
            retval = false;
        }
        Logger.recordOutput("Shooter/AtSpeed", retval );
        return retval;
    }
    
    public boolean isLightCurtainBlocked()
    {
        // reverse logic for blocked
        if ( m_beamBreak.get() == true )
        {
            return false;
        }
        else
        {
            return true;
        }
    }    
}

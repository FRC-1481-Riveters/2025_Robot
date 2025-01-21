package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.*;
import edu.wpi.first.util.CircularBuffer;

import edu.wpi.first.wpilibj.DigitalInput;


import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax m_rollerMotor = new SparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR, SparkLowLevel.MotorType.kBrushless );
    private SparkMaxConfig m_rollerMotorConfig  = new SparkMaxConfig();
    private TalonSRX m_camMotor;
    private CANCoder m_camCANCoder;
    private double m_camSetPoint;
    private boolean m_camPidEnabled;   
    private double m_camPosition; 
    private CircularBuffer <Double> circularBuffer;

    private SparkRelativeEncoder m_rollerEncoder = (SparkRelativeEncoder) m_rollerMotor.getEncoder();
    private SparkClosedLoopController m_rollerPid = m_rollerMotor.getClosedLoopController();
    private DigitalInput m_BeamBreakShooter = new DigitalInput(0);
    private DigitalInput m_BeamBreakLoaded = new DigitalInput(1);
    private boolean m_BeamBreakLoadedPrevious;
    private RobotContainer m_robotContainer;

    private boolean m_rollerPidEnabled;
    public double m_rollerRpm;
    public double m_rollerRpmSetpoint;

    public IntakeSubsystem( RobotContainer robotContainer ) 
    {
         m_robotContainer = robotContainer;
        //m_rollerMotor.restoreFactoryDefaults();
        m_rollerMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80, 30);
        m_rollerMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0.0005, 0.00000060,0.0001,0.000145);
             //0.00018
        circularBuffer = new CircularBuffer <Double> (20);

        m_camCANCoder = new CANCoder(IntakeConstants.INTAKE_CAM_CANCODER);        
        m_camCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_camCANCoder.setPosition(m_camCANCoder.getAbsolutePosition());
        m_camMotor = new TalonSRX(IntakeConstants.INTAKE_CAM_MOTOR);
        m_camMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_camMotor.configRemoteFeedbackFilter(m_camCANCoder, 0);
        // Configure Talon SRX output and sensor direction
        m_camMotor.setSensorPhase(true);
        m_camMotor.setInverted(true);
        // Set Motion Magic gains in slot0
        m_camMotor.selectProfileSlot(0, 0);
        m_camMotor.config_kF(0, IntakeConstants.INTAKE_CAM_MOTOR_KF);
        m_camMotor.config_kP(0, IntakeConstants.INTAKE_CAM_MOTOR_KP);
        m_camMotor.config_kI(0, IntakeConstants.INTAKE_CAM_MOTOR_KI);
        m_camMotor.config_kD(0, IntakeConstants.INTAKE_CAM_MOTOR_KD);
        // Set acceleration and cruise velocity
        m_camMotor.configMotionCruiseVelocity(IntakeConstants.INTAKE_CAM_MOTOR_CRUISE );
        m_camMotor.configMotionAcceleration(IntakeConstants.INTAKE_CAM_MOTOR_ACCELERATION );
        // Set extend motion limits
        m_camMotor.configForwardSoftLimitThreshold(IntakeConstants.INTAKE_CAM_MOTOR_MAX*(4096/360));
        m_camMotor.configForwardSoftLimitEnable(true);
        m_camMotor.configReverseSoftLimitThreshold(IntakeConstants.INTAKE_CAM_MOTOR_MIN*(4096/360));
        m_camMotor.configReverseSoftLimitEnable(true);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Intake/RollerJog", 0.0 );
        Logger.recordOutput("Intake/RollerSetRPM", 0.0 );
        Logger.recordOutput("Intake/RollerRPM", 0.0 );
        Logger.recordOutput("Intake/CamJog", 0.0 );
        Logger.recordOutput("Intake/CamSetpoint", 0.0 );
        Logger.recordOutput("Intake/CamPosition", 0.0 );
        Logger.recordOutput("Intake/Output", 0.0 );
        Logger.recordOutput("Intake/BeamBreakShooter", false );
        Logger.recordOutput("Intake/BeamBreakLoaded", false );
    }

    public void setIntakeRoller( double minus_one_to_one )
    {
        m_rollerPidEnabled = false;
        m_rollerMotor.set(minus_one_to_one);
        Logger.recordOutput("Intake/RollerJog", minus_one_to_one );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeRollerSpeed( double rpm )
    {
        m_rollerPid.setReference(rpm, ControlType.kVelocity);
        m_rollerRpmSetpoint = rpm;
        if( rpm > 10 )
            m_rollerPidEnabled = true;
        else
            m_rollerPidEnabled = true;

        Logger.recordOutput("Intake/RollerSetRPM", rpm );
        System.out.println( "setIntakeRollerSpeed " + rpm );
    }

    public double getRollerSpeed( )
    {
       return m_rollerRpm;
    }

    public void setCamJog( double speed )
    {
      m_camPidEnabled = false;    
      m_camMotor.set(TalonSRXControlMode.PercentOutput,speed);
      Logger.recordOutput("Intake/CamJog", speed);
      System.out.println("setCamJog " + speed );
    }

    public void setCamPosition (double position)
    {
        position = position * (4096/360);
        m_camSetPoint = position;
        m_camPidEnabled = true;
        m_camMotor.set(TalonSRXControlMode.MotionMagic,position);
        Logger.recordOutput("Intake/CamSetpoint", position);
        System.out.println("setCamPosition " + position);
    }

    @Override
    public void periodic() 
    {
        m_rollerRpm = m_rollerEncoder.getVelocity();
        m_camPosition = m_camCANCoder.getAbsolutePosition();
        Logger.recordOutput("Intake/BeamBreakShooter", !m_BeamBreakShooter.get() );
    }

    public boolean isIntakeBeamBreakLoaded()
    {
        if( m_BeamBreakLoaded.get() )
            return false;
        else
            return true;
    }
    public boolean isIntakeBeamBreakShooter()
    {
        if( m_BeamBreakLoaded.get() )
            return false;
        else
            return true;
    }
}

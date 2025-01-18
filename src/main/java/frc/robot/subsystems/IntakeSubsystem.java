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
    private TalonSRX m_angleMotor;
    private TalonSRX m_angleMotorFollower;
    private TalonSRX m_camMotor;
    private CANCoder m_camCANCoder;
    private double m_camSetPoint;
    private boolean m_camPidEnabled;   
    private double m_camPosition; 
    private CircularBuffer <Double> circularBuffer;
    private boolean m_angleStable;

    private SparkRelativeEncoder m_rollerEncoder = (SparkRelativeEncoder) m_rollerMotor.getEncoder();
    private SparkClosedLoopController m_rollerPid = m_rollerMotor.getClosedLoopController();
    private CANCoder m_angleCANcoder = new CANCoder(IntakeConstants.INTAKE_ANGLE_CANCODER);
    private double m_angleSetpoint;
    private double m_anglePosition;
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

        m_angleCANcoder.setPosition(m_angleCANcoder.getAbsolutePosition());
        m_angleMotor = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR);
        m_angleMotor.configFactoryDefault();
        // Set peak current
        m_angleMotor.setInverted(true);
        m_angleMotor.configPeakCurrentLimit(30, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.configContinuousCurrentLimit(25, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.enableCurrentLimit(true);
        m_angleMotor.setNeutralMode(NeutralMode.Coast);

        m_angleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_angleMotor.configRemoteFeedbackFilter(m_angleCANcoder, 0);
        // Configure Talon SRX output and sensor direction
        m_angleMotor.setSensorPhase(true);
        // Set Motion Magic gains in slot0
        m_angleMotor.selectProfileSlot(0, 0);
        m_angleMotor.config_kF(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KF);
        m_angleMotor.config_kP(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KP);
        m_angleMotor.config_kI(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KI);
        m_angleMotor.config_kD(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KD);
        // Set acceleration and cruise velocity
        m_angleMotor.configMotionCruiseVelocity(IntakeConstants.INTAKE_ANGLE_MOTOR_CRUISE );
        m_angleMotor.configMotionAcceleration(IntakeConstants.INTAKE_ANGLE_MOTOR_ACCELERATION );
        // Set extend motion limits
        m_angleMotor.configForwardSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MAX*(4096/360));
        m_angleMotor.configForwardSoftLimitEnable(true);
        m_angleMotor.configReverseSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MIN*(4096/360));
        m_angleMotor.configReverseSoftLimitEnable(true);


        m_angleMotorFollower = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR_FOLLOWER);
        m_angleMotorFollower.configFactoryDefault();
        // Set peak current
        m_angleMotorFollower.configPeakCurrentLimit(30, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.configContinuousCurrentLimit(25, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.enableCurrentLimit(true);
        m_angleMotorFollower.setNeutralMode(NeutralMode.Coast);
        m_angleMotorFollower.setInverted(false);
        m_angleMotorFollower.follow(m_angleMotor);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Intake/RollerJog", 0.0 );
        Logger.recordOutput("Intake/RollerSetRPM", 0.0 );
        Logger.recordOutput("Intake/RollerRPM", 0.0 );
        Logger.recordOutput("Intake/AngleSet", 0.0 );
        Logger.recordOutput("Intake/AnglePosition", 0.0 );
        Logger.recordOutput("Intake/AngleAtPosition",false);
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


    public void setIntakeAngle( double angle )
    {
        double sensorSetpoint;
        sensorSetpoint = angle * (4096/360);
        m_angleSetpoint = angle;
        m_angleMotor.set(ControlMode.MotionMagic, sensorSetpoint );
        if( angle > (0.9 * IntakeConstants.INTAKE_FLOOR_PICKUP) )
           m_robotContainer.setBling(0, 0, 0);
        Logger.recordOutput("Intake/AngleSet", angle );
        System.out.println("setIntakeAngle " + angle + ", current angle=" + getIntakeAngle());
    }

    public double getIntakeAngle ()
    {
        return (m_angleCANcoder.getAbsolutePosition());
    }

    public boolean atSetpoint(){
        double intended, current;
        boolean retval;
        intended = m_angleSetpoint;
        current = m_angleCANcoder.getAbsolutePosition();
        if( Math.abs(intended - current ) < IntakeConstants.INTAKE_ANGLE_TOLERANCE )
        {
            retval = true;
        }
        else
        {
            retval = false;
        }
        Logger.recordOutput("Intake/AngleAtPosition", retval);
        if( m_camPidEnabled == true )
        {
            if( Math.abs( m_camSetPoint - m_camPosition) < IntakeConstants.INTAKE_CAM_ANGLE_TOLERANCE)
            {
                retval = false;
            }
        }
        return retval;
    }

    public void intakeAngleDisable(boolean stopped)
    {
        m_angleMotor.set(ControlMode.PercentOutput, 0);
        System.out.println("intakeAngleDisable current angle=" + getIntakeAngle());
    }

    public double getRollerSpeed(){
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

    public boolean isAngleStable()
    {
        return m_angleStable;
    }

    @Override
    public void periodic() 
    {
        m_rollerRpm = m_rollerEncoder.getVelocity();
        m_camPosition = m_camCANCoder.getAbsolutePosition();
        boolean m_BeamBreakLoadedNew;
        m_anglePosition = m_angleCANcoder.getAbsolutePosition();

        circularBuffer.addFirst(m_anglePosition);
        double min = circularBuffer.get(0);
        double max = min;
        for(int i = 0; i<circularBuffer.size(); i++)
        {
            double val = circularBuffer.get(i);
            if(val<min)
            {
                min = val;
            }
            if(val>max)
            {
                max = val;
            }
        }
        if(max-min<1)
        {
            m_angleStable = true;
        }
        else
        {
            m_angleStable = false;
        }
        
        m_BeamBreakLoadedNew = !m_BeamBreakLoaded.get();
        if (m_BeamBreakLoadedPrevious != m_BeamBreakLoadedNew)
        {
            if (m_BeamBreakLoadedNew == true)
            {
                // note in
                m_robotContainer.setBling(255, 25, 0 );
            }
            else
            {
                // note empty
                m_robotContainer.setRosie();
            }
            m_BeamBreakLoadedPrevious = m_BeamBreakLoadedNew;
        }

        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/RollerRPM", m_rollerRpm );
        m_angleMotorFollower.follow(m_angleMotor);  // Recommended by CTRE in case follower loses power

        Logger.recordOutput("Intake/AnglePosition", m_anglePosition);
        Logger.recordOutput("Intake/CamPosition", m_camPosition);
        Logger.recordOutput("Intake/Output", m_angleMotor.getMotorOutputPercent());
        Logger.recordOutput("Intake/Current", m_angleMotor.getStatorCurrent());
        Logger.recordOutput("Intake/CurrentFollower", m_angleMotorFollower.getStatorCurrent());
        Logger.recordOutput("Intake/BeamBreakShooter", !m_BeamBreakShooter.get() );
        Logger.recordOutput("Intake/BeamBreakLoaded", m_BeamBreakLoadedNew );
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

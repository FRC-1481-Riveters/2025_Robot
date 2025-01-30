package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.CircularBuffer;

import edu.wpi.first.wpilibj.DigitalInput;


import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

   private final TalonFX m_intakeMotor =  new TalonFX(IntakeConstants.INTAKE_MOTOR, "CANivore");

    private final CANcoder m_CANCoder = new CANcoder(IntakeConstants.INTAKE_CANCODER, "CANivore");
    private final boolean m_CANCoderReversed;
    private final double m_CANCoderOffsetDegrees;
    
    private final PIDController intakePidController;

    public static final double intake_kA = 0.12872;
    public static final double intake_kV = 2.3014;
    public static final double intake_kS = 0.55493;
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward( intake_kS, intake_kV, intake_kA );
    InvertedValue intakeMotorInverted = InvertedValue.CounterClockwise_Positive; //check direction for drive (is true the same as clockwise / counter-clockwise)
    
    intakePidController = new PIDController(IntakeConstants.INTAKE_MOTOR_KP, IntakeConstants.INTAKE_MOTOR_KI, IntakeConstants.INTAKE_MOTOR_KD);
    intakePidController.enableContinuousInput(-Math.PI, Math.PI);
   
    private DigitalInput m_BeamBreakShooter = new DigitalInput(0);
    private DigitalInput m_BeamBreakLoaded = new DigitalInput(1);
    private boolean m_BeamBreakLoadedPrevious;
    private RobotContainer m_robotContainer;

    private boolean intakePidControllerEnabled;
    public double m_rollerRpm;
    public double m_rollerRpmSetpoint;

    private boolean m_hasCoral;

    public IntakeSubsystem( RobotContainer robotContainer ) 
    {
         m_robotContainer = robotContainer;

         
        MotorOutputConfigs intakeMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs intakeMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    
    SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration();
    currentConfig.currentLimit = 30;
    currentConfig.enable = true;

    intakeMotorOutputConfigs
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(intakeMotorInverted);
    intakeMotorCurrentLimitsConfigs
        .withSupplyCurrentLimit(15)
        .withSupplyCurrentLimitEnable(true);
    currentConfig.currentLimit = 12.5;
    // intakeMotor.configVoltageCompSaturation(12.5);
    // intakeMotor.enableVoltageCompensation(true);
    m_intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_intakeMotor.getConfigurator().apply(intakeMotorCurrentLimitsConfigs);
    m_intakeMotor.getConfigurator().apply(intakeMotorOutputConfigs);

           // .pidf(0.0005, 0.00000060,0.0001,0.000145);
            //0.00018
        //circularBuffer = new CircularBuffer <Double> (20);

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
        intakePidControllerEnabled = false;
        m_intakeMotor.set(minus_one_to_one);
        Logger.recordOutput("Intake/RollerJog", minus_one_to_one );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeRollerSpeed( double rpm )
    {
        intakePidController.setReference(rpm, ControlType.kVelocity);
        m_rollerRpmSetpoint = rpm;
        if( rpm > 10 )
            intakePidControllerEnabled = true;
        else
            intakePidControllerEnabled = true;

        Logger.recordOutput("Intake/RollerSetRPM", rpm );
        System.out.println( "setIntakeRollerSpeed " + rpm );
    }

    public double getRollerSpeed( )
    {
       return m_rollerRpm;
    }


    @Override
    public void periodic() 
    {
        m_rollerRpm = m_rollerEncoder.getVelocity();
        Logger.recordOutput("Intake/BeamBreakShooter", !m_BeamBreakShooter.get() );
    }

    public boolean isIntakeBeamBreakLoaded()
    {
        if( m_BeamBreakLoaded.get() )
            return false;
        else
            return true;
    }

    public void setCone( boolean bHasCone )
    {
        m_hasCoral = bHasCone;
        Logger.recordOutput("HasCone", bHasCone );
        System.out.println("setCone "  + bHasCone);
    }

    public boolean isIntakeBeamBreakShooter()
    {
        if( m_BeamBreakLoaded.get() )
            return false;
        else
            return true;
    }
}

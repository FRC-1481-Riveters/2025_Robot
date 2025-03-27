package frc.robot.subsystems;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Value;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
 
    private TalonSRX m_deploy = new TalonSRX(ClimbConstants.DEPLOY_MOTOR);
    private CANcoder m_deploy_cancoder = new CANcoder(ClimbConstants.CANCoder);
    private double m_deploySetpoint;
    private double m_deployPosition;
    private double m_climbSetpoint;
    private double m_climbPosition;
    private ElevatorSubsystem m_elevatorSubsystem;
    private final TalonFX m_climb = new TalonFX(ClimbConstants.CLIMB_MOTOR, "rio");
    private final PIDController climbPidController = new PIDController(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D);

    public ClimbSubsystem( ) 
    {
        m_deploy.configFactoryDefault();
        m_deploy.setNeutralMode(NeutralMode.Brake);
        m_deploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, ClimbConstants.TALON_TIMEOUT_MS);
        m_deploy.configMotionCruiseVelocity(ClimbConstants.DEPLOY_SPEED);
        

        m_deploy.config_kP(2, ClimbConstants.DEPLOY_P);
        m_deploy.config_kI(2, ClimbConstants.DEPLOY_I);
        m_deploy.config_kD(2, ClimbConstants.DEPLOY_D);
        m_deploy.setSensorPhase(false);

        m_deploy.configPeakCurrentLimit(30);//30
        m_deploy.configPeakCurrentDuration(20);
        m_deploy.configContinuousCurrentLimit(25);
        m_deploy.enableCurrentLimit(true);
        

        MotorOutputConfigs elevatorMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs elevatorMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
      
        elevatorMotorOutputConfigs
            .withNeutralMode(NeutralModeValue.Coast);
        elevatorMotorCurrentLimitsConfigs
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLimitEnable(true);
            //.withStatorCurrentLimit(180)
            //.withStatorCurrentLimitEnable(true);
        // elevatorMotor.enableVoltageCompensation(true);
        m_climb.getConfigurator().apply(new TalonFXConfiguration());
        m_climb.getConfigurator().apply(elevatorMotorCurrentLimitsConfigs);
        m_climb.getConfigurator().apply(elevatorMotorOutputConfigs);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Climb/Position", 0.0 );
        Logger.recordOutput("Climb/Output", 0.0 );
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        double pos = m_deploy.getSelectedSensorPosition();
        //nt_deploy_pos.setDouble( pos );
        m_climbPosition = m_climb.getPosition().getValueAsDouble();

        // This method will be called once per scheduler run
        Logger.recordOutput("Climb/Position", m_climbPosition );
        Logger.recordOutput("Climb/Output", m_climbSetpoint );
        Logger.recordOutput("Deploy/Output", m_deploySetpoint );
    }

    public void DeployClimb(double value)
    {
        System.out.println("DeployClimb ");
        if (m_deploy_cancoder.getPosition().getValueAsDouble() < ClimbConstants.DEPLOY_STOP){
        m_deploy.set(ControlMode.PercentOutput, value);
       // nt_deploy_set.setDouble( ClimbConstants.DEPLOY_SPEED );
        }
    }

    public void ClimbClimb(double speed)
    {
        System.out.println("ClimbClimb " + m_climbPosition);
        if (m_climbPosition < ClimbConstants.DEPLOY_STOP)
            speed = 0;

        m_climb.set(speed);
    }
 }
package frc.robot.subsystems;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;



import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
 
    private TalonSRX m_deploy = new TalonSRX(ClimbConstants.DEPLOY_MOTOR);
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

        MotorOutputConfigs elevatorMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs elevatorMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
      
        elevatorMotorOutputConfigs
            .withNeutralMode(NeutralModeValue.Brake);
        elevatorMotorCurrentLimitsConfigs
            .withSupplyCurrentLimit(2)
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
        m_deployPosition = m_deploy.getSelectedSensorPosition();
        m_climbPosition = m_climb.getPosition().getValueAsDouble();

        // This method will be called once per scheduler run
        Logger.recordOutput("Climb/Position", m_climbPosition );
        Logger.recordOutput("Climb/Output", m_climbSetpoint );
        Logger.recordOutput("Deploy/Output", m_deploySetpoint );
    }

    public void DeployClimb()
    {
        System.out.println("DeployClimb");
        m_deploy.setSelectedSensorPosition(ClimbConstants.DEPLOY_STOP);
    }

    public void ClimbClimb()
    {
        System.out.println("ClimbClimb");
        m_climb.set(ClimbConstants.CLIMB_SPEED);
        //m_climb.setPosition(ClimbConstants.CLIMB_STOP);
    }
 }
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.Logger;


public class WristSubsystem extends SubsystemBase {

    private TalonSRX m_wristMotor;
    private TalonSRX m_wristMotorFollower;
    private CANCoder m_cancoder;
    private ShuffleboardTab tab;
    GenericEntry kP;
    private GenericEntry kI, kD, kCruise, kAcceleration;
    private GenericEntry nt_wrist_pos, nt_wrist_set;
    private double wristPosition;

    public WristSubsystem() 
    {
        m_wristMotorFollower = new TalonSRX(WristConstants.WRIST_MOTOR_FOLLOWER);
        m_wristMotor = new TalonSRX(WristConstants.WRIST_MOTOR);
        m_wristMotorFollower.follow(m_wristMotor);
        //m_wristMotorFollower.setInverted(InvertType.InvertMotorOutput);

        m_cancoder = new CANCoder(WristConstants.WRIST_MOTOR_CANCODER);
        tab = Shuffleboard.getTab("Wrist");
        kP = tab.add("kP", WristConstants.WRIST_MOTOR_KP).getEntry();
        kI = tab.add("kI", WristConstants.WRIST_MOTOR_KI).getEntry();
        kD = tab.add("kD", WristConstants.WRIST_MOTOR_KD).getEntry();
        kCruise = tab.add("wrist_speed", WristConstants.WRIST_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("wrist_acceleration", WristConstants.WRIST_MOTOR_ACCELERATION).getEntry();
        nt_wrist_pos = tab.add("wrist_pos",0).getEntry();
        nt_wrist_set = tab.add("wrist_set",0).getEntry();
         
        m_wristMotor.configFactoryDefault();
        m_wristMotor.setNeutralMode(NeutralMode.Brake);
        m_wristMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_wristMotor.configFeedbackNotContinuous(true, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.configRemoteFeedbackFilter(m_cancoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_wristMotor.setSensorPhase(false);
        //m_wristMotor.setInverted(InvertType.InvertMotorOutput);

        // Set peak current
        m_wristMotor.configPeakCurrentLimit(20, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.configPeakCurrentDuration(200, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.configContinuousCurrentLimit(15, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_wristMotor.config_kF(1, WristConstants.WRIST_MOTOR_KF_STOW, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kP(1, kP.getDouble(WristConstants.WRIST_MOTOR_KP_STOW), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kI(1, kI.getDouble(WristConstants.WRIST_MOTOR_KI_STOW), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kD(1, kD.getDouble(WristConstants.WRIST_MOTOR_KD_STOW), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kF(0, WristConstants.WRIST_MOTOR_KF, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kP(0, kP.getDouble(WristConstants.WRIST_MOTOR_KP), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kI(0, kI.getDouble(WristConstants.WRIST_MOTOR_KI), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.config_kD(0, kD.getDouble(WristConstants.WRIST_MOTOR_KD), WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.selectProfileSlot(0, 0);
        // Set acceleration and cruise velocity
        m_wristMotor.configMotionCruiseVelocity(WristConstants.WRIST_MOTOR_CRUISE, WristConstants.TALON_TIMEOUT_MS);
        m_wristMotor.configMotionAcceleration(WristConstants.WRIST_MOTOR_ACCELERATION, WristConstants.TALON_TIMEOUT_MS);
        // Set wrist motion limits
        m_wristMotor.configForwardSoftLimitThreshold(WristConstants.WRIST_POSITION_MAX);
        m_wristMotor.configForwardSoftLimitEnable(true);
        m_wristMotor.configReverseSoftLimitThreshold(WristConstants.WRIST_POSITION_MIN);
        m_wristMotor.configReverseSoftLimitEnable(true);

        m_wristMotorFollower.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() 
    {
        double pos = m_wristMotor.getSelectedSensorPosition();
        nt_wrist_pos.setDouble( pos );
        Logger.getInstance().recordOutput("WristPosition", pos );
        Logger.getInstance().recordOutput("WristSetPoint", wristPosition);
        Logger.getInstance().recordOutput("WristOutput", nt_wrist_set.getDouble(0) );
    }

    public void setWrist( double minus_one_to_one )
    {
        m_wristMotor.set(ControlMode.PercentOutput, minus_one_to_one);
        nt_wrist_set.setDouble( minus_one_to_one );
    }
    
    public void setPosition(double value){
        wristPosition = value;
        if( value > WristConstants.WRIST_POSITION_BOUNCY )
        {
            System.out.println("wrist setPosition: using PID slot 1 for pos " + value);
            m_wristMotor.selectProfileSlot(1, 0);
        }
        else
        {
            System.out.println("wrist setPosition: using PID slot 0 for pos " + value);
            m_wristMotor.selectProfileSlot(0, 0);
        }
        m_wristMotor.setIntegralAccumulator(0);
        m_wristMotor.set(ControlMode.MotionMagic, value);
        nt_wrist_set.setDouble( wristPosition );
    }

    public double getPosition(){
        return (nt_wrist_pos.getDouble(0));
    }

    public double getWristOutput()
    {
        return( nt_wrist_set.getDouble( 0 ) );
    }
    
    public boolean atPosition()
    {
        return ( Math.abs( nt_wrist_pos.getDouble(0) - wristPosition ) < WristConstants.WRIST_TOLERANCE );
    }

}

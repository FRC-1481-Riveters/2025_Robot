package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.Logger;

public class ShoulderSubsystem extends SubsystemBase {

    private TalonSRX m_shoulderMotor;
    private TalonSRX m_shoulderMotorFollower;
    private ShuffleboardTab tab;
    GenericEntry kP;
    private GenericEntry kI, kD, kCruise, kAcceleration;
    private GenericEntry nt_shoulder_pos, nt_shoulder_set;
    private double shoulderPosition;
    private double startingPosition;

    public ShoulderSubsystem() 
    {
        m_shoulderMotorFollower = new TalonSRX(ShoulderConstants.SHOULDER_MOTOR_FOLLOWER);
        m_shoulderMotor = new TalonSRX(ShoulderConstants.SHOULDER_MOTOR);
        m_shoulderMotorFollower.follow(m_shoulderMotor);

        tab = Shuffleboard.getTab("Shoulder");
        kP = tab.add("kP", ShoulderConstants.SHOULDER_MOTOR_KP).getEntry();
        kI = tab.add("kI", ShoulderConstants.SHOULDER_MOTOR_KI).getEntry();
        kD = tab.add("kD", ShoulderConstants.SHOULDER_MOTOR_KD).getEntry();
        kCruise = tab.add("shoulder_speed", ShoulderConstants.SHOULDER_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("shoulder_acceleration", ShoulderConstants.SHOULDER_MOTOR_ACCELERATION).getEntry();
        nt_shoulder_pos = tab.add("shoulder_pos",0).getEntry();
        nt_shoulder_set = tab.add("shoulder_set",0).getEntry();
         
        m_shoulderMotor.configFactoryDefault();
        m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
        m_shoulderMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Absolute, 0, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Absolute, 1, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configFeedbackNotContinuous(true, ShoulderConstants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_shoulderMotor.setSensorPhase(false);
        // Set peak current
        m_shoulderMotor.configPeakCurrentLimit(15, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configPeakCurrentDuration(200, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configContinuousCurrentLimit(10, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_shoulderMotor.config_kF(2, ShoulderConstants.SHOULDER_MOTOR_KF, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kP(2, ShoulderConstants.SHOULDER_MOTOR_KP_MID, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kI(2, ShoulderConstants.SHOULDER_MOTOR_KI_MID, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kD(2, ShoulderConstants.SHOULDER_MOTOR_KD_MID, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kF(1, ShoulderConstants.SHOULDER_MOTOR_KF, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kP(1, ShoulderConstants.SHOULDER_MOTOR_KP_LOW, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kI(1, ShoulderConstants.SHOULDER_MOTOR_KI_LOW, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kD(1, ShoulderConstants.SHOULDER_MOTOR_KD_LOW, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kF(0, ShoulderConstants.SHOULDER_MOTOR_KF, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kP(0, kP.getDouble(ShoulderConstants.SHOULDER_MOTOR_KP), ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kI(0, kI.getDouble(ShoulderConstants.SHOULDER_MOTOR_KI), ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kD(0, kD.getDouble(ShoulderConstants.SHOULDER_MOTOR_KD), ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.selectProfileSlot(0, 0);
        // Set acceleration and cruise velocity
        m_shoulderMotor.configMotionCruiseVelocity(ShoulderConstants.SHOULDER_MOTOR_CRUISE, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configMotionAcceleration(ShoulderConstants.SHOULDER_MOTOR_ACCELERATION, ShoulderConstants.TALON_TIMEOUT_MS);
        // Set shoulder motion limits
        m_shoulderMotor.configForwardSoftLimitEnable(true);
        m_shoulderMotor.configForwardSoftLimitThreshold(ShoulderConstants.SHOULDER_MOTOR_MAX);
        m_shoulderMotor.configReverseSoftLimitEnable(true);
        m_shoulderMotor.configReverseSoftLimitThreshold(ShoulderConstants.SHOULDER_MOTOR_MIN);

        m_shoulderMotorFollower.configFactoryDefault();
        m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
        // Set peak current
        m_shoulderMotor.configPeakCurrentLimit(30, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configPeakCurrentDuration(200, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configContinuousCurrentLimit(25, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.enableCurrentLimit(true);
        
        m_shoulderMotorFollower.setNeutralMode(NeutralMode.Brake);
        }

    @Override
    public void periodic() 
    {
        double pos = m_shoulderMotor.getSelectedSensorPosition();
        nt_shoulder_pos.setDouble( pos );
        Logger.getInstance().recordOutput("ShoulderPosition", pos );
        Logger.getInstance().recordOutput("ShoulderSetPoint", shoulderPosition);
        Logger.getInstance().recordOutput("ShoulderOutput", nt_shoulder_set.getDouble(0) );
   
    }  
    public void selectShoulderPID( int range )
    {
        if( range == 0 )    // high
        {
            System.out.println("shoulder PID slot 0");
            m_shoulderMotor.selectProfileSlot(0, 0);
        }
        else if( range == 1 ) // low
        {
            System.out.println("shoulder PID slot 1");
            m_shoulderMotor.selectProfileSlot(1, 0);
        }
        else                // middle
        {
            System.out.println("shoulder PID slot 2");
            m_shoulderMotor.selectProfileSlot(2, 0);
        }
    }

    public void setShoulder( double minus_one_to_one )
    {
        m_shoulderMotor.set(ControlMode.PercentOutput, minus_one_to_one);
        nt_shoulder_set.setDouble( minus_one_to_one );
    }
    
    public double getShoulderOutput()
    {
        return( nt_shoulder_set.getDouble( 0 ) );
    }
    
    public void setPosition(double value){
        shoulderPosition = value;
        System.out.println("Shoulder setPosition " + value);
        if( shoulderPosition > ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL )
        {
          // low
          selectShoulderPID(1);
        }
        else if( shoulderPosition >= ShoulderConstants.SHOULDER_POSITION_HIGH_PRO )
        {
          // high
          selectShoulderPID(0);
        }
        else
        {
          // middle
          selectShoulderPID(2);
        }
        m_shoulderMotor.setIntegralAccumulator(0);
        m_shoulderMotor.set(ControlMode.MotionMagic, value);
        nt_shoulder_set.setDouble( shoulderPosition );
    }
    
    public boolean atPosition()
    {
        return ( Math.abs( nt_shoulder_pos.getDouble(0) - shoulderPosition ) < ShoulderConstants.SHOULDER_TOLERANCE );
    }

    public double getPosition(){
        return (nt_shoulder_pos.getDouble(0));
    }

    public void latchStartingPosition()
    {
        startingPosition = nt_shoulder_pos.getDouble(0);
    }

    public boolean startedBelowLevel()
    {
        boolean rc;

        if( startingPosition > ShoulderConstants.SHOULDER_POSITION_LEVEL )
          rc =true;
        else
          rc = false;

        return rc;
    }

    public double getCosine(){
        double cosine;
        double pos;
        pos = nt_shoulder_pos.getDouble(0);
      // if the arm is above level treat it as level (no feed forward needed to hold it in place)
        if (pos < ShoulderConstants.SHOULDER_POSITION_LEVEL) 
            pos = ShoulderConstants.SHOULDER_POSITION_LEVEL;
      //Convert arm Position in encoder counts to radians 
            cosine = ((pos - ShoulderConstants.SHOULDER_POSITION_LEVEL) / (ShoulderConstants.SHOULDER_POSITION_DOWN - ShoulderConstants.SHOULDER_POSITION_LEVEL)) * Math.PI/2.0;   
        cosine = Math.cos(cosine);
    return (cosine);
    }
}
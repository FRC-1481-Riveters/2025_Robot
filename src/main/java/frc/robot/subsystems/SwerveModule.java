package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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

//import com.ctre.phoenix6.hardware.TalonFX;
public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetDegrees;

    public static final double drive_kA = 0.12872;
    public static final double drive_kV = 2.3014;
    public static final double drive_kS = 0.55493;
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward( drive_kS, drive_kV, drive_kA );


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetDegrees, boolean absoluteEncoderReversed) {

        SupplyCurrentLimitConfiguration currentConfig;

        InvertedValue driveMotorInverted = driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //check direction for drive (is true the same as clockwise / counter-clockwise)
        InvertedValue turningMotorInverted = turningMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //check direction for turn (is true the same as clockwise / counter-clockwise)

        this.absoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "CANivore");
                
        driveMotor = new TalonFX(driveMotorId, "CANivore");
        turningMotor = new TalonFX(turningMotorId, "CANivore");

        currentConfig = new SupplyCurrentLimitConfiguration();
        currentConfig.currentLimit = 30;
        currentConfig.enable = true;

        MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs driveMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveMotorOutputConfigs
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(driveMotorInverted);
        driveMotorCurrentLimitsConfigs
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLimitEnable(true);
        currentConfig.currentLimit = 12.5;
        // driveMotor.configVoltageCompSaturation(12.5);
        // driveMotor.enableVoltageCompensation(true);
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(driveMotorCurrentLimitsConfigs);
        driveMotor.getConfigurator().apply(driveMotorOutputConfigs);

        
        MotorOutputConfigs turningMotorOutputConfigs = new MotorOutputConfigs();
        CurrentLimitsConfigs turningMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
        turningMotorOutputConfigs
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(turningMotorInverted);
        turningMotorCurrentLimitsConfigs
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLimitEnable(true);
        currentConfig.currentLimit = 15;
        // turningMotor.configVoltageCompSaturation(12.5);
        // turningMotor.enableVoltageCompensation(true);
        turningMotor.getConfigurator().apply(new TalonFXConfiguration());
        turningMotor.getConfigurator().apply(turningMotorCurrentLimitsConfigs);
        turningMotor.getConfigurator().apply(turningMotorOutputConfigs);
        
        // absoluteEncoder.configSensorInitializationStrategy( SensorInitializationStrategy.BootToAbsolutePosition );   Ensure that these exist as defaults w/ phoenix 6
        // absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        double position;
        position = driveMotor.getPosition().getValueAsDouble();  // Double check positions
        position = position * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        position = position * ModuleConstants.kWheelDiameterMeters * Math.PI;

        return( position );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),new Rotation2d(getTurningPosition()));
    }


    public double getTurningPosition() {
        double position;
        position = turningMotor.getPosition().getValueAsDouble(); // Double check position accuracy

        // convert the encodercount to radians
        position = position * (2*Math.PI) / (Constants.ModuleConstants.SWERVE_STEERING_RATIO);

        return( position );
    }

    public double getDriveVelocity() {
        double velocity;
        velocity = driveMotor.getVelocity().getValueAsDouble();

        velocity = (velocity *10) * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        velocity = velocity * ModuleConstants.kWheelDiameterMeters * Math.PI;

        return( velocity );
    }

    public double getTurningVelocity() {
        double velocity;
        velocity = turningMotor.getVelocity().getValueAsDouble();

        // convert degrees/100 milliseconds to radians per second
        velocity = (velocity * 10) * (Math.PI / 180);

        return( velocity );
    }

    public double getDriveCurrent()
    {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }
    public double getTurningCurrent()
    {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getAbsoluteEncoderDegrees() {
        double position = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        SmartDashboard.putNumber("absoluteEncoder" + absoluteEncoder.getDeviceID() + "]", position);
        position = position - absoluteEncoderOffsetDegrees;
        return position * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        double absPosition;
        System.out.println("resetEncoders");
        // Clear the drive motor encoder position
        driveMotor.setPosition( 0 );

        absPosition = getAbsoluteEncoderDegrees() * 1.0;  // negative because turning motors are upside down in mk4i
        absPosition = (absPosition/180.0) * (2048.0*Constants.ModuleConstants.SWERVE_STEERING_RATIO/2.0);
        System.out.println("Absolute Position = " + absPosition);
        turningMotor.setPosition( absPosition );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) 
    {
        double driveOutput;
        double motorOutput;

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double speed = state.speedMetersPerSecond;
        double turningPosition = getTurningPosition();
        driveOutput = speed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        driveMotor.set( driveOutput );
//        motorOutput = MathUtil.clamp( m_feedForward.calculate( speed ), -12, 12 );
//        driveMotor.set(ControlMode.PercentOutput,  motorOutput / 12.0 );
        turningMotor.set( turningPidController.calculate( turningPosition, state.angle.getRadians() ) );
    }

    public void stop() {
        driveMotor.set( 0 );
        turningMotor.set( 0 );
    }

    public void setRampRate(double rampSeconds){
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(rampSeconds)); //Ensure that control type is correct (Voltage, Torqe, DutyCycle)
    }
}
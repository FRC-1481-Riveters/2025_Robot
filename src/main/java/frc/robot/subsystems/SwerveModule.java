package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

//import com.ctre.phoenix6.hardware.TalonFX;
public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetDegrees;

    public static final double drive_kA = 0.12872;
    public static final double drive_kV = 2.3014;
    public static final double drive_kS = 0.55493;
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward( drive_kS, drive_kV, drive_kA );


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetDegrees, boolean absoluteEncoderReversed) {

        SupplyCurrentLimitConfiguration currentConfig;

        this.absoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId, "CANivore");
                
        driveMotor = new TalonFX(driveMotorId, "CANivore");
        turningMotor = new TalonFX(turningMotorId, "CANivore");

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        currentConfig = new SupplyCurrentLimitConfiguration();
        currentConfig.currentLimit = 30;
        currentConfig.enable = true;

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode( NeutralMode.Brake );
        driveMotor.configVoltageCompSaturation(12.5);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configSupplyCurrentLimit( currentConfig );

        turningMotor.configFactoryDefault();
        turningMotor.setNeutralMode( NeutralMode.Brake );
        currentConfig.currentLimit = 15;
        turningMotor.configVoltageCompSaturation(12.5);
        turningMotor.enableVoltageCompensation(true);
        turningMotor.configSupplyCurrentLimit( currentConfig );
    
        absoluteEncoder.configSensorInitializationStrategy( SensorInitializationStrategy.BootToAbsolutePosition );
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        turningMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        double position;
        position = driveMotor.getSelectedSensorPosition();  // 0..2048 counts per revolution
        position = (position / 2048) * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        position = position * ModuleConstants.kWheelDiameterMeters * Math.PI;

        return( position );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),new Rotation2d(getTurningPosition()));
    }


    public double getTurningPosition() {
        double position;
        position = turningMotor.getSelectedSensorPosition();

        // convert the encodercount to radians
        position = position * (2*Math.PI) / (2048 * Constants.ModuleConstants.SWERVE_STEERING_RATIO);

        return( position );
    }

    public double getDriveVelocity() {
        double velocity;
        velocity = driveMotor.getSelectedSensorVelocity();

        velocity = (velocity / 204.8) * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        velocity = velocity * ModuleConstants.kWheelDiameterMeters * Math.PI;

        return( velocity );
    }

    public double getTurningVelocity() {
        double velocity;
        velocity = turningMotor.getSelectedSensorPosition();

        // convert degrees/100 milliseconds to radians per second
        velocity = (velocity * 10) * (Math.PI / 180);

        return( velocity );
    }

    public double getDriveCurrent()
    {
        return driveMotor.getStatorCurrent();
    }
    public double getTurningCurrent()
    {
        return driveMotor.getStatorCurrent();
    }

    public double getAbsoluteEncoderDegrees() {
        double position = absoluteEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("absoluteEncoder" + absoluteEncoder.getDeviceID() + "]", position);
        position = position - absoluteEncoderOffsetDegrees;
        return position * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        double absPosition;
        System.out.println("resetEncoders");
        // Clear the drive motor encoder position
        driveMotor.setSelectedSensorPosition( 0 );

        absPosition = getAbsoluteEncoderDegrees() * 1.0;  // negative because turning motors are upside down in mk4i
        absPosition = (absPosition/180.0) * (2048.0*Constants.ModuleConstants.SWERVE_STEERING_RATIO/2.0);
        System.out.println("Absolute Position = " + absPosition);
        turningMotor.setSelectedSensorPosition( absPosition );
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
        driveMotor.set(ControlMode.PercentOutput,  driveOutput );
//        motorOutput = MathUtil.clamp( m_feedForward.calculate( speed ), -12, 12 );
//        driveMotor.set(ControlMode.PercentOutput,  motorOutput / 12.0 );
        turningMotor.set( ControlMode.PercentOutput, 
                          turningPidController.calculate( turningPosition, state.angle.getRadians() ) );
    }

    public void stop() {
        driveMotor.set( ControlMode.PercentOutput, 0 );
        turningMotor.set( ControlMode.PercentOutput, 0 );
    }

    public void setRampRate(double rampSeconds){
        driveMotor.configOpenloopRamp(rampSeconds);
    }
}
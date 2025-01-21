package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LocalADStarAK;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.gyroPort, "CANivore");

    private double yawOffset = 0;
    private double pitchOffset = 0;

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(),
                    backLeft.getPosition(), backRight.getPosition() });

    private final Field2d m_field = new Field2d();
    private Pose2d savedOdometry;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setYaw(0, 1000);
                zeroHeading(180);
            } catch (Exception e) {
            }
        }).start();

        // Configure AutoBuilder
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()),
                (speeds, feedForwards) -> driveRobotRelative(speeds),
                DriveConstants.kDriveController,
                DriveConstants.kRobotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    public void zeroHeading(double heading) {
        // gyro.setAccumZAngle(0); //.setFusedHeading(0);
        yawOffset = gyro.getYaw().getValueAsDouble() + heading;
        System.out.println("zeroHeading: heading=" + heading + ", offset=" + yawOffset);
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] { frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() },
                new Pose2d(0, 0, getRotation2d()));
    }

    public double getHeading() {
        // TODO: not sure if the Pigeon2 getYaw returning -368 to 368 is OK here (should
        // it be 0...360)?
        // return Math.IEEEremainder( gyro.getYaw() - yawOffset, 360 );
        return gyro.getYaw().getValueAsDouble() - yawOffset;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] { frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() }, pose);
    }

    public void saveOdometry() {
        savedOdometry = getPose();
    }

    public Pose2d getSavedOdometry() {
        return savedOdometry;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition() });
        Logger.recordOutput("Drive/Gyro", getRotation2d());
        Logger.recordOutput("Drive/FLDriveCurrent", frontLeft.getDriveCurrent());
        Logger.recordOutput("Drive/FLTurningCurrent", frontLeft.getTurningCurrent());
        Logger.recordOutput("Drive/FRDriveCurrent", frontRight.getDriveCurrent());
        Logger.recordOutput("Drive/FRTurningCurrent", frontRight.getTurningCurrent());
        Logger.recordOutput("Drive/BLDriveCurrent", backLeft.getDriveCurrent());
        Logger.recordOutput("Drive/BLTurningCurrent", backLeft.getTurningCurrent());
        Logger.recordOutput("Drive/BRDriveCurrent", backRight.getDriveCurrent());
        Logger.recordOutput("Drive/BRTurningCurrent", backRight.getTurningCurrent());
        m_field.setRobotPose(odometer.getPoseMeters());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        Logger.recordOutput("SwerveStates/Setpoints", desiredStates);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        return states;
    }

}

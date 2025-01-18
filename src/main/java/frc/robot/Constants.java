package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.825);
        public static final double SWERVE_STEERING_RATIO = (150.0 / 7.0);
        public static final double kPTurning = 0.2;

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // FIXME: patch these motor IDs up to match the Swervie 2022 configuration
        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 18;
        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackRightDriveMotorPort = 19;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kFrontRightTurningMotorPort = 15;
        public static final int kBackLeftTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 12;

        public static final int gyroPort = 60;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 17;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 14;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset = 0.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = 0.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffset = 0.0;
        public static final double kBackRightDriveAbsoluteEncoderOffset = 0.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0292; // MK4i 16.5 FPS => 5.0292 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0.0, 0.00), // Translation constants
                new PIDConstants(10, 0.0, 0.0), // Rotation constants
                kPhysicalMaxSpeedMetersPerSecond,
                new Translation2d(kWheelBase / 2, kTrackWidth / 2).getNorm(), // Drive base radius (distance from center
                                                                              // to furthest module)
                new ReplanningConfig());

        public static final double DRIVE_DIVIDER_NORMAL = 2.0;
        public static final double DRIVE_DIVIDER_TURBO = 1.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_CAM_MOTOR = 56;
        public static final int INTAKE_CAM_CANCODER = 57;
        public static final int INTAKE_ROLLER_MOTOR = 58;
        public static final int INTAKE_ANGLE_MOTOR = 59;
        public static final int INTAKE_ANGLE_MOTOR_FOLLOWER = 62;
        public static final int INTAKE_ANGLE_CANCODER = 61;
        public static final int TALON_TIMEOUT_MS = 5000;

        public static final double INTAKE_ROLLER_SPEED_TOLERANCE = 50;
        public static final double INTAKE_ROLLER_SPEED_AMP = 300;
        public static final double INTAKE_ROLLER_SPEED_SPEAKER = 1500;

        public static final double INTAKE_CAM_ANGLE_TOLERANCE = 10;
        public static final double INTAKE_CAM_SPEAKER = 10.0;
        public static final double INTAKE_CAM_3FOOT = 190.0;
        public static final double INTAKE_CAM_MOTOR_ACCELERATION = 4000;
        public static final double INTAKE_CAM_MOTOR_CRUISE = 2000;
        public static final double INTAKE_CAM_MOTOR_KP = 7;
        public static final double INTAKE_CAM_MOTOR_KI = 0;
        public static final double INTAKE_CAM_MOTOR_KD = 0.06;
        public static final double INTAKE_CAM_MOTOR_KF = 0;
        public static final double INTAKE_CAM_MOTOR_MAX = 200;
        public static final double INTAKE_CAM_MOTOR_MIN = 5;

        public static final double INTAKE_ANGLE_STOWED = 10;
        public static final double INTAKE_FLOOR_PICKUP = 189;
        public static final double INTAKE_SOURCE = 45.0;
        public static final double INTAKE_HALF = 115;
        public static final double INTAKE_ANGLE_TOLERANCE = 7;
        public static final double INTAKE_ANGLE_MOTOR_ACCELERATION = 3500;
        public static final double INTAKE_ANGLE_MOTOR_CRUISE = 8000;
        public static final double INTAKE_ANGLE_MOTOR_KP = 0.5;
        public static final double INTAKE_ANGLE_MOTOR_KI = 0;
        public static final double INTAKE_ANGLE_MOTOR_KD = 0.06;
        public static final double INTAKE_ANGLE_MOTOR_KF = 0;
        public static final double INTAKE_ANGLE_MOTOR_MAX = 191;
        public static final double INTAKE_ANGLE_MOTOR_MIN = 0;

    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_TOP_FORWARD = 3;
        public static final int SHOOTER_MOTOR_TOP_BACK = 4;
        public static final int SHOOTER_MOTOR_BOTTOM_FORWARD = 5;
        public static final int SHOOTER_MOTOR_BOTTOM_BACK = 6;
        public static final double SHOOTER_SPEED_TOLERANCE = 0.1;
        public static final double SHOOTER_SPEED_SPEAKER = 3600;
        public static final double SHOOTER_SPEED_3FOOT = 4200;
        public static final double SHOOTER_SPEED_PODIUM = 4800;
        public static final double SHOOTER_SPEED_AMP = 6000;
        public static final double SHOOTER_SPEED_AMP_LOAD = 800;
        public static final double SHOOTER_SPEED_AUTON_SPEW = 2700;

    }

    public static final class ShooterPivotConstants {
        public static final int SHOOTER_PIVOT_MOTOR = 36;
        public static final int SHOOTER_PIVOT_CANCODER = 37;

        public static final double SHOOTER_PIVOT_MIN = 10;
        public static final double SHOOTER_PIVOT_AMP = 23.0;
        public static final double SHOOTER_PIVOT_TRAVEL = 58.0;
        public static final double SHOOTER_PIVOT_CLEAR_INTAKE = 61.0;
        public static final double SHOOTER_PIVOT_CLOSE = 127.0; // 126.5
        public static final double SHOOTER_PIVOT_CLIMB = 85.0;
        public static final double SHOOTER_PIVOT_3FOOT = 114.0; // 3 deg diff
        public static final double SHOOTER_PIVOT_PODIUM = 106.5; // 3 deg diff?
        public static final double SHOOTER_PIVOT_AMP_LOAD = 88.2;
        public static final double SHOOTER_PIVOT_MAX = 140; // max travel

        public static final double SHOOTER_PIVOT_ACCELERATION = 500;
        public static final double SHOOTER_PIVOT_VELOCITY = 250;
        public static final double SHOOTER_PIVOT_0_KP = 0.0095; // 0.004
        public static final double SHOOTER_PIVOT_0_KI = 0.024;
        public static final double SHOOTER_PIVOT_0_KD = 0.00034;
        public static final double SHOOTER_PIVOT_CLOSE_KP = 0.015; // 0.004;
        public static final double SHOOTER_PIVOT_CLOSE_KI = 0.03;// 0.0300;
        public static final double SHOOTER_PIVOT_CLOSE_KD = 0.00040;
        public static final double SHOOTER_PIVOT_AMP_KP = 0.0028; // 0.005;
        public static final double SHOOTER_PIVOT_AMP_KI = 0.0450;
        public static final double SHOOTER_PIVOT_AMP_KD = 0.00034;
        // public static final double SHOOTER_PIVOT_0_KP = 0.003;
        // public static final double SHOOTER_PIVOT_0_KI = 0.004;
        // public static final double SHOOTER_PIVOT_0_KD = 0.0003;
        public static final double SHOOTER_PIVOT_0_KF = 0.0;
    }

    public static final class ElevatorConstants {
        public static final double ELEVATOR_VELOCITY = 40;
        public static final double ELEVATOR_ACCELERATION = 120;
        public static final int ELEVATOR_MOTOR = 42;
        public static final double ELEVATOR_POSITION_TOLERANCE = 0.3;
        // fill out position values later
        public static final double ELEVATOR_START = 0;
        public static final double ELEVATOR_ABOVE_BUMP = -0.2;
        public static final double ELEVATOR_PAST_BUMP = -2.7;
        public static final double ELEVATOR_WING = -7.9;
        public static final double ELEVATOR_3FOOT = -16.0;
        public static final double ELEVATOR_CLOSE = -16.0;
        public static final double ELEVATOR_PODIUM = -14.0;
        public static final double ELEVATOR_AMP_START = -5.66;
        public static final double ELEVATOR_PIVOT_CLEAR = -11.3;
        public static final double ELEVATOR_AMP = -19.5;
        public static final double ELEVATOR_CLIMB_START = -13.0;
        public static final double ELEVATOR_CLIMB_TAUT = -8.1;
        public static final double ELEVATOR_CLIMB_FULL = -3.2;
        public static final double ELEVATOR_MAX = -20.0;
        public static final double ELEVATOR_TRAP = 0;
    }

    public static final class ClimbConstants {
        public static final int CLIMB_MOTOR = 24;
        public static final int CLIMB_MOTOR_FOLLOWER = 25;
        public static final double CLIMB_POSITION_MAX = 15.0;
        public static final double CLIMB_ENCODER_FULLY_CLIMBED = -150;
        public static final int CLIMB_MIN = 0;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
        public static final int CANDLE_ID = 40;
    }
}

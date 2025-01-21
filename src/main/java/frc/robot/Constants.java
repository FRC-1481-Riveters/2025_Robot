package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = /* FIXME 1 */ 0.5 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // FIXME: patch these motor IDs up to match the Swervie 2022 configuration
        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 18;
        public static final int kBackRightDriveMotorPort = 21;

        public static final int kFrontLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 19;

        public static final int gyroPort = 60;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 14;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 17;
        public static final int kBackRightDriveAbsoluteEncoderPort = 20;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset  = 137.988;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = -44.385;
        public static final double kBackLeftDriveAbsoluteEncoderOffset   = -39.990;
        public static final double kBackRightDriveAbsoluteEncoderOffset  = 8.174;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;

        public static final double CreepLoading = -0.37;
        public static final double CreepBalance = -0.30;
        public static final double CreepBalanceMobility = 0.35;
        public static final double CreepBalanceMobilityBackup = -0.2;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = -5;
        public static final double kPYController = -5;
        public static final double kPThetaController = -2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ShoulderConstants {
        public static final int SHOULDER_MOTOR = 41;
        public static final int SHOULDER_MOTOR_FOLLOWER = 40;
        public static final double SHOULDER_MOTOR_KF = 0.00;
        public static final double SHOULDER_MOTOR_KP = 14.0; //35.0;
        public static final double SHOULDER_MOTOR_KI = 0.000;
        public static final double SHOULDER_MOTOR_KD = 0.23; //0.0;
        public static final double SHOULDER_MOTOR_KP_LOW = 10.0;
        public static final double SHOULDER_MOTOR_KI_LOW = 0.00;
        public static final double SHOULDER_MOTOR_KD_LOW = 0.40;
        public static final double SHOULDER_MOTOR_KP_MID = 10.0; //14.0;
        public static final double SHOULDER_MOTOR_KI_MID = 0.00;
        public static final double SHOULDER_MOTOR_KD_MID = 0.1; //0.23;
        public static final double SHOULDER_MOTOR_CRUISE = 200;
        public static final double SHOULDER_MOTOR_ACCELERATION = 400;
        public static final int SHOULDER_MOTOR_MIN = 1500;
        public static final int SHOULDER_MOTOR_MAX = 2600;
        public static final int SHOULDER_POSITION_TEST_85 = 2500;
        public static final int SHOULDER_POSITION_TEST_15 = 1800;
        public static final int SHOULDER_POSITION_STOWED = 2329; //2428;
        public static final int SHOULDER_POSITION_SHELF_CUBE = SHOULDER_POSITION_STOWED - 715; //1650; //1700;   
        public static final int SHOULDER_POSITION_SHELF_CONE = SHOULDER_POSITION_STOWED - 755; //1655; //1705;   
        public static final int SHOULDER_POSITION_HIGH = SHOULDER_POSITION_STOWED - 842; //1538; //1588;   
        public static final int SHOULDER_POSITION_HIGH_PRO = SHOULDER_POSITION_STOWED - 667; //1718; //1768;  
        public static final int SHOULDER_POSITION_MID = SHOULDER_POSITION_STOWED - 740; //1640; //1690;   
        public static final int SHOULDER_POSITION_MID_PRO = SHOULDER_POSITION_STOWED - 522; //1858;//1908; 
        public static final int SHOULDER_POSITION_LOW = SHOULDER_POSITION_STOWED - 83; //2297; //2347;
        public static final int SHOULDER_POSITION_LEVEL = SHOULDER_POSITION_STOWED - 400; //1861; //1911;
        public static final int SHOULDER_POSITION_DOWN = SHOULDER_POSITION_STOWED + 505; //2885; //2935;
        public static final int SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL = SHOULDER_POSITION_STOWED - 280; //2100; //2150;
        public static final int SHOULDER_POSITION_CONE_PICKUP = SHOULDER_POSITION_STOWED + 85; //2465; //2515;
        public static final int SHOULDER_POSITION_CUBE_PICKUP = SHOULDER_POSITION_STOWED + 45; //2425; //2475;
        
        public static final double SHOULDER_TOLERANCE = 40;
        public static final int TALON_TIMEOUT_MS = 5000;
    }

    public static final class WristConstants {
        public static final int WRIST_MOTOR = 51;
        public static final int WRIST_MOTOR_CANCODER = 52;
        public static final int WRIST_MOTOR_FOLLOWER = 50;
        public static final double WRIST_MOTOR_KF = 0.00;
        public static final double WRIST_MOTOR_KP = 6.0; //9.0;
        public static final double WRIST_MOTOR_KI = 0.00; //3;
        public static final double WRIST_MOTOR_KD = 1.0; //1.2;
        public static final double WRIST_MOTOR_KF_STOW = 0.0;
        public static final double WRIST_MOTOR_KP_STOW = 1.0;
        public static final double WRIST_MOTOR_KI_STOW = 0.0;
        public static final double WRIST_MOTOR_KD_STOW = 0.0;
        public static final double WRIST_MOTOR_CRUISE = 1000;
        public static final double WRIST_MOTOR_ACCELERATION = 400;
        public static final int WRIST_POSITION_MIN = 46;
        public static final int WRIST_POSITION_MAX = 1967;
        public static final int WRIST_POSITION_TEST_85 = 469;
        public static final int WRIST_POSITION_TEST_15 = 1702;
        public static final int WRIST_TOLERANCE = 50;
        public static final int WRIST_POSITION_SHELF_CONE = 955;//710;  
        public static final int WRIST_POSITION_SHELF_CUBE = 749;  
        public static final int WRIST_POSITION_HIGH = 856;    
        public static final int WRIST_POSITION_HIGH_PRO = 463;
        public static final int WRIST_POSITION_MID = 834;
        public static final int WRIST_POSITION_MID_PRO = 443;    
        public static final int WRIST_POSITION_LOW = 441; 
        public static final int WRIST_POSITION_STRAIGHT = 952;
        public static final int WRIST_POSITION_BOUNCY = 1200;
        public static final int WRIST_POSITION_STOWED = 1710;
        public static final int WRIST_POSITION_CONE_PICKUP = 165;
        public static final int WRIST_POSITION_CUBE_PICKUP = 275;

        public static final int TALON_TIMEOUT_MS = 5000;
    }
    
    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 58;
        public static final int TALON_TIMEOUT_MS = 5000;
    }

    public static final class ExtendConstants {
        public static final int EXTEND_MOTOR = 55;
        public static final int EXTEND_MOTOR_CANCODER = 56;
        public static final double EXTEND_MOTOR_KF = 0.00;
        public static final double EXTEND_MOTOR_KP = 2.0; //2.0;
        public static final double EXTEND_MOTOR_KI = 0; //0.001; //0.0001;
        public static final double EXTEND_MOTOR_KD = 0.4; //0.5;                                  
        public static final double EXTEND_MOTOR_CRUISE = 3000;
        public static final double EXTEND_MOTOR_ACCELERATION = 12000;
        public static final int EXTEND_MOTOR_MIN = 0;
        public static final int EXTEND_MOTOR_MAX = 18500; //23263;
        public static final int EXTEND_MOTOR_TEST_85 = 15725;
        public static final int EXTEND_MOTOR_TEST_15 = 2775;
        public static final int EXTEND_POSITION_SHELF = 4938;   
        public static final int EXTEND_POSITION_HIGH = 16854;   
        public static final int EXTEND_POSITION_HIGH_PRO = 16000; //16854;   
        public static final int EXTEND_POSITION_MID = 2750;   
        public static final int EXTEND_POSITION_LOW = 0;
        public static final int EXTEND_POSITION_SCORE_LOW = 1800;
        public static final int EXTEND_POSITION_CONE_PICKUP = 4020;   
        public static final int EXTEND_POSITION_CUBE_PICKUP = 4664;   
        public static final int EXTEND_TOLERANCE = 200;
        public static final int TALON_TIMEOUT_MS = 5000;
    }


    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }
}

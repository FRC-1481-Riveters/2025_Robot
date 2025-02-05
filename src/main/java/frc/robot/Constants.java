package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 17;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 14;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset = -0.214844; 
        public static final double kFrontRightDriveAbsoluteEncoderOffset = -0.246826; 
        public static final double kBackLeftDriveAbsoluteEncoderOffset = -0.157471; 
        public static final double kBackRightDriveAbsoluteEncoderOffset = -0.031006; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0292; // MK4i 16.5 FPS => 5.0292 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;

        public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );

        public static RobotConfig kRobotConfig;
        static {
            try{
            kRobotConfig = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            }
        }

        public static final double DRIVE_DIVIDER_NORMAL = 2.0;
        public static final double DRIVE_DIVIDER_TURBO = 1.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 38;

        public static final double INTAKE_ROLLER_SPEED_TOLERANCE = 50;
        public static final double INTAKE_ROLLER_SPEED_CORAL = 300;
        public static final double INTAKE_ROLLER_SPEED_ALGAE = 1500;

        public static final double INTAKE_CAM_ANGLE_TOLERANCE = 10;
        public static final double INTAKE_CAM_L1 = 10.0;
        public static final double INTAKE_CAM_L2 = 190.0;
        public static final double INTAKE_CAM_L3 = 190.0;
        public static final double INTAKE_CAM_L4 = 190.0;
        public static final double INTAKE_CAM_BARGE = 190.0;
        public static final double INTAKE_CAM_PROCESSOR = 190.0;
        public static final double INTAKE_CAM_MOTOR_ACCELERATION = 4000;
        public static final double INTAKE_CAM_MOTOR_CRUISE = 2000;
        public static final double INTAKE_MOTOR_KP = 7;
        public static final double INTAKE_MOTOR_KI = 0;
        public static final double INTAKE_MOTOR_KD = 0.06;
        public static final double INTAKE_MOTOR_KF = 0;
        public static final double INTAKE_CAM_MOTOR_MAX = 200;
        public static final double INTAKE_CAM_MOTOR_MIN = 5;

        public static final double INTAKE_ANGLE_STOWED = 10;
        public static final double INTAKE_FLOOR_PICKUP = 189;
        public static final double INTAKE_SOURCE = 45.0;
        public static final double INTAKE_HALF = 115;

    }

    public static final class ClawConstants {
        public static final int CLAW_MOTOR = 36;

        public static final double CLAW_MIN = 0;
        public static final double CLAW_TRAVEL = 58.0;
        public static final double CLAW_CLIMB = 85.0;
        public static final double CLAW_L1 = 114.0; // 3 deg diff
        public static final double CLAW_L2 = 106.5; // 3 deg diff?
        public static final double CLAW_L3 = 88.2;
        public static final double CLAW_L4 = 88.2;
        public static final double CLAW_BARGE = 88.2;
        public static final double CLAW_REEF = 88.2;
        public static final double CLAW_PROCESSOR = 88.2;
        public static final double CLAW_MAX = 40; // max travel

        public static final double CLAW_ACCELERATION = 500;
        public static final double CLAW_VELOCITY = 250;
        public static final double CLAW_0_KP = 0.0095; // 0.004
        public static final double CLAW_0_KI = 0.024;
        public static final double CLAW_0_KD = 0.00034;
        public static final double CLAW_CLOSE_KP = 0.015; // 0.004;
        public static final double CLAW_CLOSE_KI = 0.03;// 0.0300;
        public static final double CLAW_CLOSE_KD = 0.00040;
        //NEED TO CHANGE THE NAMES/CONSTANTS BELOW
        // public static final double CLAW_0_KP = 0.003;
        // public static final double CLAW_0_KI = 0.004;
        // public static final double CLAW_0_KD = 0.0003;
        public static final double CLAW_0_KF = 0.0;
    }

    public static final class ElevatorConstants {
        public static final double ELEVATOR_VELOCITY = 40;
        public static final double ELEVATOR_ACCELERATION = 120;
        public static final int ELEVATOR_MOTOR = 42;
        public static final int ELEVATOR_MOTOR_FOLLOWER = 43;
        public static final double ELEVATOR_POSITION_TOLERANCE = 0.3;
        // fill out position values later
        public static final double ELEVATOR_START = 0;
        public static final double ELEVATOR_WING =  -4;
        public static final double ELEVATOR_L1 = -16.0;
        public static final double ELEVATOR_L2 = -16.0;
        public static final double ELEVATOR_L3 = -14.0;
        public static final double ELEVATOR_L4 = -5.66;
        public static final double ELEVATOR_PIVOT_CLEAR = -11.3;
        public static final double ELEVATOR_BARGE = -19.5;
        public static final double ELEVATOR_ALGAE_LOW = -19.5;
        public static final double ELEVATOR_ALGAE_HIGH = -19.5;
        public static final double ELEVATOR_PROCESSOR = -19.5;
        public static final double ELEVATOR_CLIMB_START = -13.0;
        public static final double ELEVATOR_CLIMB_TAUT = -8.1;
        public static final double ELEVATOR_CLIMB_FULL = -3.2;
        public static final double ELEVATOR_MOTOR_KP = 0.30;
        public static final double ELEVATOR_MOTOR_KI = 0.090;
        public static final double ELEVATOR_MOTOR_KD = 0.005;
        public static final double ELEVATOR_MOTOR_KF = 0;
        public static final double ELEVATOR_MAX = -20.0;
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

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.825);
        public static final double SWERVE_STEERING_RATIO = (150.0 / 7.0);
        public static final double kPTurning = 0.2;

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
            );
    
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

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
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

        public static final double DRIVE_DIVIDER_NORMAL = 1.5;
        public static final double DRIVE_DIVIDER_SLOW = 4.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 38;
        public static final int CANRANGE = 8;

        public static final double INTAKE_ROLLER_SPEED_TOLERANCE = 50;
        public static double INTAKE_ROLLER_SPEED_CURRENT;
        public static final double INTAKE_ROLLER_SPEED_CORAL_IN = -0.3;
        public static final double INTAKE_ROLLER_SPEED_CORAL_OUT = -0.9;
        public static final double INTAKE_ROLLER_SPEED_ALGAE_IN = 1;
        public static final double INTAKE_ROLLER_SPEED_ALGAE_OUT = -10;
        public static final double INTAKE_ROLLER_SPEED_KEEP = 0.15;

        public static final double INTAKE_CAM_ANGLE_TOLERANCE = 10;
        public static final double INTAKE_MOTOR_KP = 7;
        public static final double INTAKE_MOTOR_KI = 0;
        public static final double INTAKE_MOTOR_KD = 0.06;
        public static boolean INTAKING;

    }

    public static final class ClawConstants {
        public static final int CLAW_MOTOR = 36;

        //public static final double CLAW_MIN = 0.1; -3.5
        public static final double CLAW_START = -3.3;
        public static final double CLAW_REEF = 5;
        public static final double CLAW_HIGH = 5.5;
        public static final double CLAW_BARGE = 11.6; 
        public static final double CLAW_ALGAE = 20;
        public static final double CLAW_ALGAE_TRAVEL = 18.6;
        public static final double CLAW_PROCESSOR = 25.406;
        public static final double CLAW_FLOOR = 26;
        public static final double CLAW_ALGAE_STORE = 13.012;
        //public static final double CLAW_MAX = 19.9;
        public static final double CLAW_ELEVATOR_CLEAR = 5.0; // max travel

        public static final double CLAW_ACCELERATION = 500;
        public static final double CLAW_VELOCITY = 100;
        public static final double CLAW_KP = 0.2; // 0.004
        public static final double CLAW_KI = 0.024;
        public static final double CLAW_KD = 0.000;
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
        public static final double ELEVATOR_POSITION_TOLERANCE = .75;
        // fill out position values later
        public static final double ELEVATOR_START = 1;
        public static final double ELEVATOR_L2 = 9.5;
        public static final double ELEVATOR_L3 = 21;
        public static final double ELEVATOR_L4 = 38;
        public static final double ELEVATOR_BARGE = 39.5;
        public static final double ELEVATOR_ALGAE_LOW = 13.5;
        public static final double ELEVATOR_ALGAE_HIGH = 24;
        public static final double ELEVATOR_PROCESSOR = 2.371;
        public static final double ELEVATOR_CLIMB = 21.0;
        public static final double ELEVATOR_MOTOR_ABOVE_KP = 0.015;
        public static final double ELEVATOR_MOTOR_ABOVE_KI = 0.090;
        public static final double ELEVATOR_MOTOR_ABOVE_KD = 0.000;
        public static final double ELEVATOR_MOTOR_ABOVE_KF = 0;
        public static final double ELEVATOR_MOTOR_BELOW_KP = 0.15;
        public static final double ELEVATOR_MOTOR_BELOW_KI = 0.090;
        public static final double ELEVATOR_MOTOR_BELOW_KD = 0.000;
        public static final double ELEVATOR_MOTOR_BELOW_KF = 0;
        public static final double ELEVATOR_MAX = 40.0;
    }

    public static final class ClimbConstants {
        public static final int MATCH_CURRENT = 15;
        public static final int CLIMB_CURRENT = 180; 

    }  

    public static final class VisionConstants{
    public static final String LIMELIGHT_NAME = "limelight-riveter";

    public static final double MOVE_P = 0.400000;
    public static final double MOVE_I = 0.000000;
    public static final double MOVE_D = 0.000600;
    //(0.300000, 0.000000, 0.000600, 0.01);

    public static final double ROTATE_P = 0.01000;
    public static final double ROTATE_I = 0.000000;
    public static final double ROTATE_D = 0.000100;
    //(0.03000, 0.000000, 0.001000, 0.01);

    public static final double X_REEF_ALIGNMENT_P = 0.15;
    public static final double Y_REEF_ALIGNMENT_P = 0.5;
    public static final double ROT_REEF_ALIGNMENT_P = 0.03;
    
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;  
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.4;  
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.0005;
  
    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double TOLERANCE = 0.01;
    }

    public static class reefAlignmentConstants 
    {
      public static final double reefSpacing = 0.1651;                               // 13" = 0.3302; tag is in between, so halve that
      public static final double robotWidth = Units.inchesToMeters(35.43);    // including bumpers
      public static final double shortDistance = Units.inchesToMeters(14);    // waypoint before going straight into reef
      public static final double coralScoreOffset = Units.inchesToMeters(0);  // how far off center the scoring mechanism is
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

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

        public static RobotConfig kRobotConfig;
        static {
            try{
            kRobotConfig = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            }
        }

        public static final double DRIVE_DIVIDER_NORMAL = 1.4; //  1.4 = 70% of Max Speed
        public static final double DRIVE_DIVIDER_SLOW = 6.0;  //  4.0 = 25% of Max Speed
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 38;
        public static final int CANRANGE = 8;

        public static final double INTAKE_ROLLER_SPEED_TOLERANCE = 50;
        public static double INTAKE_ROLLER_SPEED_CURRENT;
        public static final double INTAKE_ROLLER_SPEED_CORAL_IN = -0.5;
        public static final double INTAKE_ROLLER_SPEED_CORAL_OUT = -0.9;
        public static final double INTAKE_ROLLER_SPEED_ALGAE_IN = 0.7;
        public static final double INTAKE_ROLLER_SPEED_ALGAE_OUT = -10;  
        public static final double INTAKE_ROLLER_SPEED_BARGE = -1;
        public static final double INTAKE_ROLLER_SPEED_KEEP = 0.15;
        public static final double INTAKE_ROLLER_SPEED_TROUGH = 0.6;

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
        public static final double CLAW_CLIMB_CLEAR = -2.5;
        public static final double CLAW_LOW = 16.7;
        public static final double CLAW_REEF = 1.7;
        public static final double CLAW_HIGH = 2.7;
        public static final double CLAW_BARGE = 4.0;
        public static final double CLAW_FLICK = 11.2;
        public static final double CLAW_ALGAE = 16.7;
        public static final double CLAW_ALGAE_TRAVEL = 12.0;
        public static final double CLAW_PROCESSOR = 22.1;
        public static final double CLAW_FLOOR = 22.7;
        public static final double CLAW_ALGAE_STORE = 9.7;
        //public static final double CLAW_MAX = 19.9;
        public static final double CLAW_ELEVATOR_CLEAR = 1.0; //4.25; // max travel
        public static final double CLAW_AUTON_CLEAR = 0.75;//1.4

        public static final double CLAW_PID_CLAMP_HIGH = 1.0;
        public static final double CLAW_PID_CLAMP_NORMAL = 0.4;
        public static final double CLAW_ACCELERATION = 500;
        public static final double CLAW_VELOCITY = 100;
        public static final double CLAW_KP = 0.1; // 0.2
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
        public static final double ELEVATOR_POSITION_TOLERANCE = 0.5; //f.75;
        // fill out position values later
        public static final double ELEVATOR_START = 1;
        public static final double ELEVATOR_L1 = 12;
        public static final double ELEVATOR_L2 = 9.5;
        public static final double ELEVATOR_L3 = 21;
        public static final double ELEVATOR_L4 = 38.6;  // 40.6 max
        public static final double ELEVATOR_BARGE = 40;
        public static final double ELEVATOR_BARGE_SHOT = 35;
        public static final double ELEVATOR_ALGAE_LOW = 13.5;
        public static final double ELEVATOR_ALGAE_HIGH = 24;
        public static final double ELEVATOR_PROCESSOR = 2.371;
        public static final double ELEVATOR_CLIMB = 21.0;
        public static final double ELEVATOR_MOTOR_ABOVE_KP = 0.021;
        public static final double ELEVATOR_MOTOR_ABOVE_KI = 0.005;
        public static final double ELEVATOR_MOTOR_ABOVE_KD = 0.000;
        public static final double ELEVATOR_MOTOR_ABOVE_KF = 0;
        public static final double ELEVATOR_MOTOR_BELOW_KP = 0.15;
        public static final double ELEVATOR_MOTOR_BELOW_KI = 0.04;
        public static final double ELEVATOR_MOTOR_BELOW_KD = 0.000;
        public static final double ELEVATOR_MOTOR_BELOW_KF = 0;
        public static final double ELEVATOR_MOTOR_BARGE_KP = 4;
        public static final double ELEVATOR_MOTOR_BARGE_KI = 0.04;
        public static final double ELEVATOR_MOTOR_BARGE_KD = 0.000;
        public static final double ELEVATOR_MOTOR_BARGE_KF = 0;
        public static final double ELEVATOR_MAX = 40.0;
        public static final double ELEVATOR_BAR = 40.0;
    }

    public static final class ClimbConstants {
        public static final int DEPLOY_MOTOR = 53;
        public static final double DEPLOY_P = 0;
        public static final double DEPLOY_I = 0;
        public static final double DEPLOY_D = 0;
        public static final double DEPLOY_SPEED = 1;
        public static final double DEPLOY_STOP = -0.014;
        //DEPLOY_START = 0.2;
        public static final int CANCoder = 6;
        public static final int CLIMB_MOTOR = 52;
        public static final double CLIMB_P = 0;
        public static final double CLIMB_I = 0;
        public static final double CLIMB_D = 0;
        public static final double CLIMB_SPEED = -0.5;
        public static final double CLIMB_STOP = -150;
        public static final int TALON_TIMEOUT_MS = 5000;
    }

    public static final class VisionConstants{
    public static final String LIMELIGHT_NAME = "limelight-riveter";

    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double TOLERANCE = 0.01;
    }

    public static class reefAlignmentConstants 
    {
      public static final double reefSpacing = Units.inchesToMeters(13);        // space between coral posts on the same reef face
      public static final double robotWidth = Units.inchesToMeters(35.43);      // robot length including bumpers
      public static final double shortDistance = Units.inchesToMeters(14);      // waypoint before going straight into reef
      public static final double coralScoreOffset = Units.inchesToMeters(0.0);  // how far off center the scoring mechanism is
      public static final double cheatOffset = Units.inchesToMeters(2.0
      );
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

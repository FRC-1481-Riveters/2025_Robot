package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.text.DecimalFormat;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class PositionPIDCommand extends Command{
    
    private static CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = new PPHolonomicDriveController(
                    // PID constants for translation
                   new PIDConstants(3.0, 0.000006, 0, 0.3),
                    // PID constants for rotation
                    new PIDConstants(3.5, 0.000003, 0, 0.3)
                );

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    public static final Time kEndTriggerDebounce = Seconds.of(0.1);
    public static final Distance kPositionTolerance = Centimeter.of(2.0);
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(4.0);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);

            
    private PositionPIDCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(()-> {
            Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);
    
            var rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

            var speed = mSwerve.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

            DecimalFormat df = new DecimalFormat("#.00");
            System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed  
                + "  x=" + df.format(mSwerve.getState().Pose.getX()) + "  targetX=" + df.format(goalPose.getX()) 
                + "  y=" + df.format(mSwerve.getState().Pose.getY()) + " targetY=" + df.format(goalPose.getY())
                + " rot=" + df.format(mSwerve.getState().Pose.getRotation().getRadians()) + " targetRot=" + df.format(goalPose.getRotation().getRadians()));
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }
        
    public static Command generateCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose, double timeout){
        timeout = 20; //!*!*!*
        return new PositionPIDCommand(mSwerve, goalPose).withTimeout(timeout).finallyDo( () -> 
            {
                ChassisSpeeds cs = new ChassisSpeeds();
                mSwerve.driveLoop(cs);
            }
        );
    }

    @Override
    public void initialize() {
        DecimalFormat df = new DecimalFormat("#.00");
        System.out.println("PositionPIDCommand to x=" + df.format(goalPose.getX()) + " y=" + df.format(goalPose.getY() ));
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        ChassisSpeeds cs;

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

//        // Update the robot position
//        Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-riveter");
//        if( robotPose.getX() != 0 && robotPose.getY() != 0 )
//        {
//            // Don't update the position if the AprilTag isn't visible
//            mSwerve.resetPose( robotPose );
//        }

        cs = mDriveController.calculateRobotRelativeSpeeds( mSwerve.getState().Pose, goalState );
        if( cs.vxMetersPerSecond < 0 )
            cs.vxMetersPerSecond = Math.max( cs.vxMetersPerSecond,-2.5 ); //!*!*!* max speed
        else
            cs.vxMetersPerSecond = Math.min( cs.vxMetersPerSecond,2.5 ); //!*!*!* max speed
        if( cs.vyMetersPerSecond < 0 )
            cs.vyMetersPerSecond = Math.max( cs.vyMetersPerSecond,-2.5 ); //!*!*!* max speed
        else
            cs.vyMetersPerSecond = Math.min( cs.vyMetersPerSecond,2.5 ); //!*!*!* max speed

//        cs.vxMetersPerSecond /= 2;
//        cs.vyMetersPerSecond *= 1.3;

        DecimalFormat df = new DecimalFormat("#.00");
//        System.out.println("cs.x=" + df.format(cs.vxMetersPerSecond)+ " cs.y=" + df.format(cs.vyMetersPerSecond) + " cs.rot=" + df.format(cs.omegaRadiansPerSecond));

        mSwerve.driveLoop(cs);
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.fusionEnable();
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        DecimalFormat df = new DecimalFormat("#.00");
        System.out.println("PositionPIDCommand to x=" + df.format(goalPose.getX()) + " y=" + df.format(goalPose.getY() ));
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}

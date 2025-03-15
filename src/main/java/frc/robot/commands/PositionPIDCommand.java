package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
                   new PIDConstants(0.4, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(1.5, 0, 0)
                );

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    public static final Time kEndTriggerDebounce = Seconds.of(0.1);
    public static final Distance kPositionTolerance = Centimeter.of(1.0);
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
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

            System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose, double timeout){
        final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
        return new PositionPIDCommand(mSwerve, goalPose).withTimeout(timeout).finallyDo( () -> {
            mSwerve.applyRequest(() ->
                driveRequest.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0));
            }
        );
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

        mSwerve.applyRequest(()->driveRequest.withSpeeds(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getState().Pose, goalState
            )    
        ));

    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Telemetry;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightHelpers.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;


public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;
  private CommandSwerveDrivetrain m_commandSwerveDrivetrain;


  public VisionSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    m_commandSwerveDrivetrain = commandSwerveDrivetrain;
    config();
    
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    // LimelightHelpers.setCropWindow("limelight-riveter", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-riveter",
        0.3556, 
        0.1016,
        0.3429,
        0,00
        -2,3
        );
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-riveter", new int[] {3,6,7,8,9,10,11,16,17,18,19,20,21,22});
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("limelight-riveter");

      var driveState = m_commandSwerveDrivetrain.getState();
      double headingRadians = driveState.Pose.getRotation().getAngle();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-riveter", headingRadians, 0, 0, 0, 0, 0);

      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-riveter");
      if (llMeasurement != null ) {
        boolean useMegaTag2 = false; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
          LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-riveter");
          
          if(Math.abs(omegaRps) < 2.0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
          {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
              doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
              doRejectUpdate = true;
            }
          }
          if(mt1.tagCount == 0)
          {
            doRejectUpdate = true;
          }

          m_commandSwerveDrivetrain.updateOdometry(mt1.pose, doRejectUpdate, mt1.timestampSeconds,false);
        }
        else if (useMegaTag2 == true)
        {
          LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-riveter");
          if(Math.abs(omegaRps) < 2.0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if(mt2.tagCount == 0)
          {
            doRejectUpdate = true;
          }
          m_commandSwerveDrivetrain.updateOdometry(mt2.pose, doRejectUpdate, mt2.timestampSeconds,true);
      }
    }
  }

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }
}

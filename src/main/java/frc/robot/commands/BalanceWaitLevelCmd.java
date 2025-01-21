package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
public class BalanceWaitLevelCmd extends CommandBase {

private SwerveSubsystem m_Subsystem;
private double m_balanceDegrees;

    public BalanceWaitLevelCmd(SwerveSubsystem subsystem, double balanceDegrees)
    {
      m_Subsystem = subsystem;
      m_balanceDegrees = balanceDegrees;
    }
    
    @Override
    public void initialize() 
    {
      double angle;
      angle = Math.abs(m_Subsystem.getPitch());
      System.out.println("BalanceWaitLevelCmd started: angle=" + angle + ", target=" + m_balanceDegrees);
    }
  
    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angle;
    angle = Math.abs(m_Subsystem.getPitch());
    if (angle < m_balanceDegrees){
      System.out.println("BalanceWaitLevelCmd finished, angle=" + angle + ", target=" + m_balanceDegrees); 
      return true;
    }
    else{
      return false;
    }
  }
}
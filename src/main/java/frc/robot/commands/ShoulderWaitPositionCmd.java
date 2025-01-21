package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderWaitPositionCmd extends CommandBase {

    private ShoulderSubsystem m_shoulderSubsystem;
    private boolean m_bLessIfFalse;
    private double m_position;

    public ShoulderWaitPositionCmd( ShoulderSubsystem subsystem, boolean bLessIfFalse, double position )
    {
        m_shoulderSubsystem = subsystem;
        m_bLessIfFalse = bLessIfFalse;
        m_position = position;
    }
    

  @Override
  public void initialize() 
  {
    System.out.println("ShoulderWaitPositionCmd to " + m_position + ", current " + m_shoulderSubsystem.getPosition() );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      boolean retval;
      if( (m_bLessIfFalse == false) && (m_shoulderSubsystem.getPosition() < m_position + ShoulderConstants.SHOULDER_TOLERANCE) )
      {
        System.out.println("ShoulderWaitPositionCmd finished: " + m_shoulderSubsystem.getPosition() + " lt " + m_position);
        retval = true;
      }
      else if((m_bLessIfFalse == true) && m_shoulderSubsystem.getPosition() > m_position - ShoulderConstants.SHOULDER_TOLERANCE)
      {
        System.out.println("ShoulderWaitPositionCmd finished: " + m_shoulderSubsystem.getPosition() + " gt " + m_position);
        retval = true;
      }
      else
      {
        retval = false;
      }
      return retval;
  }
}
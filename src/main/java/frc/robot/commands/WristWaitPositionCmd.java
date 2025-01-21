package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristWaitPositionCmd extends CommandBase {

    private WristSubsystem m_wristSubsystem;
    private boolean m_bLessIfFalse;
    private double m_position;

    public WristWaitPositionCmd( WristSubsystem subsystem, boolean bLessIfFalse, double position )
    {
        m_wristSubsystem = subsystem;
        m_bLessIfFalse = bLessIfFalse;
        m_position = position;
    }
    
    @Override
    public void initialize() 
    {
      System.out.println("WristWaitPositionCmd to " + m_position + ", current " + m_wristSubsystem.getPosition() );
    }
  
    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      boolean retval;
      if( (m_bLessIfFalse == false) && (m_wristSubsystem.getPosition() < m_position + WristConstants.WRIST_TOLERANCE) )
      {
        System.out.println("WristWaitPositionCmd finished: " + m_wristSubsystem.getPosition() + " lt " + m_position);
        retval = true;
      }
      else if((m_bLessIfFalse == true) && m_wristSubsystem.getPosition() > m_position + WristConstants.WRIST_TOLERANCE)
      {
        System.out.println("WristWaitPositionCmd finished: " + m_wristSubsystem.getPosition() + " gt " + m_position);
        retval = true;
      }
      else
      {
        retval = false;
      }
      return retval;
  }
}
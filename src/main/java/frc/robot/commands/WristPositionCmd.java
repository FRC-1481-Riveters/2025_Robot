package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristPositionCmd extends CommandBase {

    private WristSubsystem m_wristSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;

    public WristPositionCmd( WristSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_wristSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
        addRequirements(subsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position;
    System.out.println( "WristPositionCmd to " + m_setPosition + ", current " + m_wristSubsystem.getPosition());
    m_wristSubsystem.setWrist(0);
    position = m_wristSubsystem.getPosition();
    m_wristSubsystem.setPosition(m_setPosition);
    if( m_wristSubsystem.atPosition() == false )
    {
      if( position < m_setPosition )
        m_wristSubsystem.setWrist(1.0);
      else
        m_wristSubsystem.setWrist(-1.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( m_wristSubsystem.atPosition() || m_waitAtPosition == false)
      {
        System.out.println("WristPositionCmd finished: set " + m_setPosition + ", at " + m_wristSubsystem.getPosition());
        return true;
      }
      else
      {
        return false;
      }
  }

  @Override
  public void end( boolean interrupted ) {
    if (m_setPosition < WristConstants.WRIST_POSITION_BOUNCY )
    {
      m_wristSubsystem.setPosition(m_setPosition); 
      System.out.println("WristPositionCmd end: interrupted=" + interrupted + ", set " + m_setPosition + " done, now PID");
    }
    else
    {
      m_wristSubsystem.setWrist(0);
      System.out.println("WristPositionCmd end: interrupted=" + interrupted + ", set " + m_setPosition + " done, now 0");
    }
}
}
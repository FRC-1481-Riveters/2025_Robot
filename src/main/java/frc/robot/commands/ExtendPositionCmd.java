package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.Constants.ExtendConstants;

public class ExtendPositionCmd extends CommandBase {

    private ExtendSubsystem m_extendSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;

    public ExtendPositionCmd( ExtendSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_extendSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
        addRequirements(subsystem);
      }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println( "ExtendPositionCmd init: set=" + m_setPosition + ", current " + m_extendSubsystem.getPosition());
    m_extendSubsystem.setPosition(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (m_extendSubsystem.atPosition() || m_waitAtPosition == false) )
    {
      System.out.println( "ExtendPositionCmd finished: set=" + m_setPosition + ", current " + m_extendSubsystem.getPosition());
      return true;
    }
    else
    {
      return false;
    }
  }

  @Override
  public void end( boolean interrupted )
  {
    System.out.println( "ExtendPositionCmd end: interrupted=" + interrupted + ", set " + m_setPosition + ", current " + m_extendSubsystem.getPosition());
  }
}
package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderPositionCmd extends CommandBase {

    private ShoulderSubsystem m_shoulderSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;
    private int countdown_pid_handoff;

    public ShoulderPositionCmd( ShoulderSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_shoulderSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
        addRequirements(subsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulderSubsystem.setPosition(m_setPosition);
    System.out.println("ShoulderPositionCmd: init, set " + m_setPosition + ", at " + m_shoulderSubsystem.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if( m_waitAtPosition == false )
    {
      System.out.println("ShoulderPositionCmd: don't wait, set " + m_setPosition + ", at " + m_shoulderSubsystem.getPosition());
      return true;
    }
    else if( m_shoulderSubsystem.atPosition() )
    {
      System.out.println("ShoulderPositionCmd: done, set " + m_setPosition + ", at " + m_shoulderSubsystem.getPosition());
      return true;
    }
    else
      return false;
  }
}
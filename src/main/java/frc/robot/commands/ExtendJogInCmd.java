// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.Constants.ExtendConstants;

public class ExtendJogInCmd extends CommandBase {

  private ExtendSubsystem m_extendSubsystem;

  /** Creates a new ExtendJogUp. */
  public ExtendJogInCmd( ExtendSubsystem extendSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extendSubsystem = extendSubsystem;
    addRequirements((extendSubsystem));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("extend in");
    m_extendSubsystem.setExtend( -0.4 );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_extendSubsystem.setPosition( m_extendSubsystem.getPosition() );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

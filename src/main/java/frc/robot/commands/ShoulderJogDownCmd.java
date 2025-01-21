// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderJogDownCmd extends CommandBase {

  private ShoulderSubsystem m_shoulderSubsystem;

  /** Creates a new ShoulderJogDown. */
  public ShoulderJogDownCmd( ShoulderSubsystem shoulderSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulderSubsystem = shoulderSubsystem;
    addRequirements(shoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulderSubsystem.setShoulder(0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_shoulderSubsystem.setPosition( m_shoulderSubsystem.getPosition() );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristJogUpCmd extends CommandBase {

  private WristSubsystem m_wristSubsystem;

  /** Creates a new WristJogUp. */
  public WristJogUpCmd( WristSubsystem wristSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wristSubsystem = wristSubsystem;
    addRequirements((wristSubsystem));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("wrist up");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wristSubsystem.setWrist( -1.0 );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if( m_wristSubsystem.getPosition() < WristConstants.WRIST_POSITION_BOUNCY )
      m_wristSubsystem.setPosition( m_wristSubsystem.getPosition() );
    else
      m_wristSubsystem.setWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeJogCmd extends CommandBase {

  private IntakeSubsystem m_intakeSubsystem;
  private boolean m_isLoading;


  /** Creates a new ExtendJogUp. */
  public IntakeJogCmd( IntakeSubsystem intakeSubsystem, boolean bIsLoading ) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    addRequirements((intakeSubsystem));
    m_isLoading= bIsLoading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("intake jog cmd" + m_isLoading);
    if( m_isLoading == true )
    {
      if( m_intakeSubsystem.getCone() == true )
      {
        // Load cones in with full intake speed
        m_intakeSubsystem.setIntake( 1.0 );
      }
      else
      {
        // Load cubes in a little slower (and note cube vs. cone intake is opposite direction)
        m_intakeSubsystem.setIntake( -0.5 );
      }
    }
    else
    {   
      // Depositing gamepiece
      if( m_intakeSubsystem.getCone() == true )
      {
        m_intakeSubsystem.setIntake( -0.5 );
      }
      else
      {
        m_intakeSubsystem.setIntake( 0.35 );
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if( m_intakeSubsystem.getCone() == true && m_isLoading == true )
    {
      // For cones, the intake runs the cone up into the wrist to avoid dropping it
      m_intakeSubsystem.setIntake(0.07);
    }
    else
    {
      // For cubes, you can just drop it
      m_intakeSubsystem.setIntake(0.0);
    }
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private TalonSRX m_intakeMotor;
    private boolean m_hasCone;

    public IntakeSubsystem() 
    {
        m_intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR);
          
        m_intakeMotor.configFactoryDefault();
        // Configure Talon  SRX output and sensor direction
        m_intakeMotor.setSensorPhase(false);
        // Set peak current
        m_intakeMotor.configPeakCurrentLimit(20, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeMotor.configContinuousCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeMotor.enableCurrentLimit(true);
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setIntake( double minus_one_to_one )
    {
        double output;
        output = minus_one_to_one;

        m_intakeMotor.set(ControlMode.PercentOutput, output);
        Logger.getInstance().recordOutput("IntakeSpeed", output );
    }

    public void setCone( boolean bHasCone )
    {
        m_hasCone = bHasCone;
        Logger.getInstance().recordOutput("HasCone", bHasCone );
        System.out.println("setCone "  + bHasCone);
    }
    public boolean getCone()
    {
        return( m_hasCone );
    }
}

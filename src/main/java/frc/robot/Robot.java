// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private Command m_testCommand;

    private RobotContainer m_robotContainer;

    private int m_rainbowFirstPixelHue;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        Logger.recordMetadata("ProjectName", "Mozart"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
 
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        

        System.out.println(" " + 1481);
        m_robotContainer = new RobotContainer();
        
        //CameraServer.startAutomaticCapture();

        // Limelight port forwarding (allows Limelight to be seen from outside)
        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
       // for (int port = 5800; port <= 5807; port++) {
            //PortForwarder.add(port, "limelight.local", port);
  //}
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
       
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Turn off position control when the robot is disabled.
        m_robotContainer.driverJoystick.getHID().setRumble(RumbleType.kBothRumble,0);
        m_robotContainer.operatorJoystick.getHID().setRumble(RumbleType.kBothRumble,0);
       
    }

    @Override
    public void disabledPeriodic() {
        int i;
        for( i=0; i < m_robotContainer.m_ledBuffer.getLength(); i++ )
        {
            //Calculate the hue - hue is easier for rainbows because the color
            //shape is a circle so only one value needs to process
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_robotContainer.m_ledBuffer.getLength())) % 180;
            //Set the value
            m_robotContainer.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_robotContainer.m_led.setData(m_robotContainer.m_ledBuffer);
        //Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        //Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() 
    {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.StopControls(true);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) 
        {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.StopControls(true);
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }
    

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_testCommand = m_robotContainer.getTestCommand();
        m_robotContainer.m_allTestsPassed = true;

        // schedule the autonomous command (example)
        if (m_testCommand != null) 
        {
            m_testCommand.schedule();
        }
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit(){
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.StopControls(true);
    }
}

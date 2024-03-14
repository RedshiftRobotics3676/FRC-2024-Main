// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Robot(double period) {
    super(period);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // m_robotContainer.leds.rainbow();
    // m_robotContainer.leds.pulseAllianceColor();
    // m_robotContainer.leds.setSolidRGB(0, 0, 0);
    // m_robotContainer.leds.updateLEDs();
    // SmartDashboard.putNumber("FL Module rotation?",m_robotContainer.drivetrain.getModule(0).getPosition(true).angle.getRotations());
    // m_robotContainer.drivetrain.getModule(0).getPosition(true).angle.getDegrees();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.arm.removeDefaultCommand();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // m_robotContainer.drivetrain.setEstimatedPose(m_robotContainer.vision.getEstimatedGlobalPose());
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.arm.setDefaultCommand(m_robotContainer.arm.setArmPosition(0.035));
  }

  @Override
  public void teleopPeriodic() {
    // m_robotContainer.drivetrain.setEstimatedPose(m_robotContainer.vision.getEstimatedGlobalPose());
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

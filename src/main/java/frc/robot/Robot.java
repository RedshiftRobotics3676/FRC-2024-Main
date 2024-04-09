// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.armDefaultPos;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Timer autonTimer;
  private int counter = 0;

  Robot(double period) {
    super(period);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    autonTimer = new Timer();
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

      // m_robotContainer.drivetrain.setEstimatedPose(m_robotContainer.vision.getEstimatedGlobalPose());

  }

  @Override
  public void disabledInit() {
    // m_robotContainer.leds.runFromChooser();
  }

  @Override
  public void disabledPeriodic() {
    counter++;
    if (counter >= 7) {
      m_robotContainer.leds.pulseAllianceColor();
      counter = 0;
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonTimer.restart();
    m_robotContainer.arm.removeDefaultCommand();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.leds.runFromChooser();
    SmartDashboard.putNumber("auton timer", autonTimer.get());
    // m_robotContainer.drivetrain.setEstimatedPose(m_robotContainer.vision.getEstimatedGlobalPose());
  }

  @Override
  public void autonomousExit() {
    autonTimer.stop();
    m_robotContainer.intake.stop();
    m_robotContainer.intake.stopShooter();
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.leds.solidAllianceColor();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.arm.setDefaultCommand(m_robotContainer.arm.setArmPosition(armDefaultPos));
  }

  @Override
  public void teleopPeriodic() {
    // m_robotContainer.drivetrain.setEstimatedPose(m_robotContainer.vision.getEstimatedGlobalPose());

    m_robotContainer.leds.runFromChooser();
    
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

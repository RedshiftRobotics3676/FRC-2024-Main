// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator;
  /** Creates a new Elavator. */
  public Elevator() {
    elevator = new TalonFX(ElevatorConstants.kElevatorPort);
  }

  // public Command up(double speed) {
  //   return new RunCommand(() -> elevator.setControl(new MotionMagicVelocityVoltage(speed)), this);
  // }

  // public Command down(double speed) {
  //   return new RunCommand(() -> elevator.setControl(new MotionMagicVelocityVoltage(-speed)), this);
  // }

  public Command upOutput(double speed) {
    return new RunCommand(() -> elevator.setControl(new DutyCycleOut(speed)), this);
  }

  public Command downOutput(double speed) {
    return new RunCommand(() -> elevator.setControl(new DutyCycleOut(-speed)), this);
  }

  public void up2(double speed) {
    elevator.setControl(new DutyCycleOut(speed));
  }

  public void down2(double speed) {
    elevator.setControl(new DutyCycleOut(-speed));
  }

  public Command set(double speed) {
    return runOnce(() -> elevator.setControl(new DutyCycleOut(speed)));
  }

  public void set2(double speed) {
    elevator.setControl(new DutyCycleOut(speed));
  }

  public Command stop() {
    return new RunCommand(() -> elevator.setControl(new DutyCycleOut(0)), this);
  }

  public void stop2() {
    elevator.setControl(new DutyCycleOut(0));
  }

  public Command setSoftLimits(boolean enable) {
    return runOnce(() -> elevator.getConfigurator().apply(ElevatorConstants.kElevatorSoftLimitSwitch.withForwardSoftLimitEnable(enable).withReverseSoftLimitEnable(enable)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

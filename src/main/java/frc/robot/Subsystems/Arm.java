// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private TalonFX armLeader;
  private TalonFX armFollower;

  private MotionMagicVelocityVoltage motionMagicVelocity;
  private MotionMagicVoltage motionMagicVoltage;
  private Follower follower;

  private CANcoder canCoder;
  
  /** Creates a new Arm. */
  public Arm() {
    armLeader = new TalonFX(ArmConstants.kArmLeaderPort);
    armFollower = new TalonFX(ArmConstants.kArmFollowerPort);

    armLeader.getConfigurator().apply(ArmConstants.kArmConfig, 5);
    armFollower.getConfigurator().apply(ArmConstants.kArmConfig, 5);

    motionMagicVelocity = new MotionMagicVelocityVoltage(0).withOverrideBrakeDurNeutral(true);
    motionMagicVoltage = new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true);
    follower = new Follower(ArmConstants.kArmLeaderPort, true);

    canCoder = new CANcoder(ArmConstants.kArmCanCoderPort);
  }

  public Command setArmSpeed(double speed) {
    return run(() -> {
                  armLeader.setControl(motionMagicVelocity.withVelocity(speed));
                  armFollower.setControl(follower);
                });
  }

  public Command setArmPosition(double position) {
    return run(() -> {
                  armLeader.setControl(motionMagicVoltage.withPosition(position));
                  armFollower.setControl(follower);
                });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", canCoder.getPosition().getValue());
  }
}
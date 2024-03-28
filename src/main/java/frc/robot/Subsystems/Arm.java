// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.RobotConstants.armDefaultPos;

import java.util.ArrayList;
import java.util.Collections;

public class Arm extends SubsystemBase {
  private TalonFX armLeader;
  private TalonFX armFollower;

  private Follower follower;

  private CANcoder canCoder;

  private double lastSetPos = 0;

  private ArrayList<Double> array;

  private TalonFXSimState armLeaderSim;
  private TalonFXSimState armFollowerSim;

  private CANcoderSimState canCoderSim;

  private SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1), 
    174.2222222222222, 
    SingleJointedArmSim.estimateMOI(0.75, 5), 
    0.75, 
    0, 
    Units.rotationsToRadians(0.255), 
    true, 
    Units.rotationsToRadians(0));

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              0,//Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  
  /** Creates a new Arm. */
  public Arm() {
    armLeader = new TalonFX(kArmLeaderPort);
    armFollower = new TalonFX(kArmFollowerPort);

    armLeader.getConfigurator().apply(kArmConfigs);
    armFollower.getConfigurator().apply(kArmConfigs);

    // motionMagicVoltage = new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true).withEnableFOC(false);
    // motionMagicVelocity = new MotionMagicVelocityVoltage(1).withOverrideBrakeDurNeutral(true).withEnableFOC(false);
    follower = new Follower(kArmLeaderPort, true);

    armFollower.setControl(follower);

    canCoder = new CANcoder(kArmCanCoderPort);
    canCoder.getConfigurator().apply(kArmCANCoderConfigs);

    armLeaderSim = armLeader.getSimState();
    armFollowerSim = armFollower.getSimState();

    canCoderSim = canCoder.getSimState();

    armLeaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    armFollowerSim.Orientation = ChassisReference.Clockwise_Positive;

    canCoderSim.Orientation = ChassisReference.Clockwise_Positive;

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // double[] helperArray = new double[50];
    // Arrays.fill(helperArray, 0.035);
    array = new ArrayList<Double>(50);
    Collections.fill(array, 0.035);
    System.out.println(array.size());
    for (int i = 0; i < 50; i++) {
      array.add(i, 0.035);
    }
    System.out.println(array.size());
  }

  // public Command setArmSpeed(double speed) {
  //   return run(() -> {
  //                 armLeader.setControl(motionMagicVelocity.withVelocity(speed));
  //                 armFollower.setControl(follower);
  //               });
  // }

  public Command setArmPosition(double position) {
    return run(() -> {
                  // lastSetPos = position;
                  armLeader.setControl(motionMagicVoltage.withPosition(position));
                  armFollower.setControl(follower);
                });
  }

  public Command setArmPositionEnd(double position) {
    return runEnd(() -> {
                  // lastSetPos = position;
                  armLeader.setControl(motionMagicVoltage.withPosition(position));
                  armFollower.setControl(follower);
                },
                () -> {
                  armLeader.setControl(motionMagicVoltage.withPosition(armDefaultPos));
                  armFollower.setControl(follower);
                });
  }

  public Command setArmPositionCheck(double position) {
    return new FunctionalCommand(
                () -> {
                  armLeader.setControl(motionMagicVoltage.withPosition(position));
                  armFollower.setControl(follower);
                },
                () -> {

                },
                (interrupted) -> {

                },
                () -> {
                  return canCoder.getAbsolutePosition().getValueAsDouble() > position - 0.005 && canCoder.getAbsolutePosition().getValueAsDouble() < position + 0.005;
                  // return armLeader.getMotionMagicIsRunning().getValue().equals(MotionMagicIsRunningValue.Enabled);
                },
                this);
  }

  public Command setArmPositionOnce(double position) {
    return runOnce(() -> {
                  // lastSetPos = position;
                  armLeader.setControl(motionMagicVoltage.withPosition(position));
                  armFollower.setControl(follower);
                });
  }

  public Command setArmPositionAverage(Double position) {
    return run(() -> {
                  SmartDashboard.putNumber("guitar arm input pos", position);
                  SmartDashboard.putNumber("remove", array.remove(0));
                  array.add(position);
                  System.out.println(position);
                  double sum = 0;
                  for (double pos : array) {
                    sum += pos;
                  }
                  double finalPos = sum/50;
                  SmartDashboard.putNumber("guitar arm pos", finalPos);

                  armLeader.setControl(motionMagicVoltage.withPosition(finalPos));
                  armFollower.setControl(follower);
                });
  }

  public Command incrementArmPosition(double amount) {
    return run(() -> {
                  System.out.println(lastSetPos + " " + (amount/10.0) + " " + amount);
                  lastSetPos += amount/10.0;
                  armLeader.setControl(motionMagicVoltage.withPosition(lastSetPos));
                  armFollower.setControl(follower);
                });
  }

  // public Command positionDutyCycle(double position) {
  //   return run(() -> {
  //                 armLeader.setControl(new PositionDutyCycle(position));
  //                 armFollower.setControl(follower);
  //               });
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", canCoder.getAbsolutePosition().getValue());
  }

  @Override
    public void simulationPeriodic() {
      armSim.setInput(armLeader.getMotorVoltage().getValueAsDouble());
      armSim.update(0.02);

      armLeaderSim.setRotorVelocity(Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec()) / 60);
      armFollowerSim.setRotorVelocity(Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec()) / 60);

      canCoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()) - 0.192383);

      m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    }
}

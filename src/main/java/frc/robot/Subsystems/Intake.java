// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;
  private TalonFX shooterMotor;
  private DutyCycleOut dutyCycleOut;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new TalonSRX(10);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    shooterMotor = new TalonFX(9);
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);

    // TalonFX Request type to control the shooter motor
    dutyCycleOut = new DutyCycleOut(1).withEnableFOC(false).withOverrideBrakeDurNeutral(true);
  }

  /**
   * @param speed Percent output to set the intake motor to
   * @return Command to run the intake motor and stop it when the command is interupted
   */
  public Command in(double speed) {
    return runEnd(() -> intakeMotor.set(ControlMode.PercentOutput, speed), // Runnable to run the intake motor
                  () -> intakeMotor.set(ControlMode.PercentOutput, 0)); // Runnable to stop the intake motor when the command is interupted
  }

  /**
   * @param speed Percent output to set the intake motor to in reverse
   * @return Command to run the intake motor in reverse and stop it when the command is interupted
   */
  public Command out(double speed) {
    return runEnd(() -> intakeMotor.set(ControlMode.PercentOutput, -speed), // Runnable to run the intake motor in reverse
                  () -> intakeMotor.set(ControlMode.PercentOutput, 0)); // Runnable to stop the intake motor when the command is interupted
  }

  public Command shoot() {
    return new FunctionalCommand(
        () -> {
            // Start the shooter motor at full speed
            shooterMotor.setControl(dutyCycleOut);
            SmartDashboard.putBoolean("started", true);
            SmartDashboard.putBoolean("stopped", false);
        },
        () -> {
          SmartDashboard.putBoolean("waiting", true);
            // Wait until the shooter motor is at full speed
            if (shooterMotor.getVelocity().refresh().getValue() >= 100) {
              SmartDashboard.putBoolean("waiting", false);
              SmartDashboard.putBoolean("shooting", true);
                // Start the intake motor at full speed to feed the disc into the shooter
                intakeMotor.set(ControlMode.PercentOutput, 1);
            }
        },
        (interrupted) -> {
          SmartDashboard.putBoolean("shooting", false);
          SmartDashboard.putBoolean("started", false);
          SmartDashboard.putBoolean("stopped", true);
            // Stop both motors
            shooterMotor.stopMotor();
            intakeMotor.set(ControlMode.PercentOutput, 0);
        },
        () -> {
            // This command never finishes on its own, but it can be interrupted
            return false;
        }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

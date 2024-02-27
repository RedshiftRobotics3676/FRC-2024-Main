// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;
  private TalonFX shooterMotor;
  private DutyCycleOut dutyCycleOut;

  private ColorSensorV3 colorSensor;

  // private TalonSRXSimCollection intakeSim;
  // private TalonFXSimState shooterSim;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new TalonSRX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);

    shooterMotor = new TalonFX(kShooterMotorID);
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);

    shooterMotor.getConfigurator().apply(kShooterConfigs);

    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    // TalonFX Request type to control the shooter motor
    // dutyCycleOut = new DutyCycleOut(-0.6).withEnableFOC(false).withOverri deBrakeDurNeutral(true);
  
    // intakeSim = intakeMotor.getSimCollection();
  
  }

  /**
   *  
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

  public Command inAutoStop(double speed) {
    return new FunctionalCommand(
      () -> {
          intakeMotor.set(ControlMode.PercentOutput, speed);
      }, 
      () -> {
        
      }, 
      (interrupted) -> {
          intakeMotor.set(ControlMode.PercentOutput, 0);
      }, 
      () -> {
          return colorSensor.getProximity() > 1500;
      }, 
      this
    );
  }

  public Command shoot() {
    return new FunctionalCommand(
        () -> {
            // Start the shooter motor at full speed
            shooterMotor.setControl(motionMagicVelocity.withVelocity(1));
            SmartDashboard.putBoolean("started", true);
            SmartDashboard.putBoolean("stopped", false);
        },
        () -> {
          SmartDashboard.putBoolean("waiting", true);
            // Wait until the shooter motor is at full speed
            if (shooterMotor.getVelocity().getValue() >= 0.5) {
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
        },
        this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("colorProx", colorSensor.getProximity());
    SmartDashboard.putString("color", colorSensor.getColor().toString());
  }
}

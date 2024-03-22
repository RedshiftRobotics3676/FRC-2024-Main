// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
// import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.IntakeOut;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor;
  private TalonFX shooterMotor;

  private TalonFXSimState shooterSim;

  private FlywheelSim flywheelSim;

  // private ColorSensorV3 colorSensor;

  // private TalonSRXSimCollection intakeSim;
  // private TalonFXSimState shooterSim;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new TalonSRX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);

    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 500));

    shooterMotor = new TalonFX(kShooterMotorID);
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);

    shooterMotor.getConfigurator().apply(kShooterConfigs);

    shooterSim = shooterMotor.getSimState();
    shooterSim.Orientation = ChassisReference.Clockwise_Positive;

    flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.01);

    // colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    // TalonFX Request type to control the shooter motor
    // dutyCycleOut = new DutyCycleOut(-0.6).withEnableFOC(false).withOverri deBrakeDurNeutral(true);
  
    // intakeSim = intakeMotor.getSimCollection();
  
  }

  public Command set(double percent) {
    return runOnce(() -> intakeMotor.set(ControlMode.PercentOutput, percent));
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

  public Command inThenOut(double speed) {
    return runEnd(() -> intakeMotor.set(ControlMode.PercentOutput, speed), // Runnable to run the intake motor
                  () -> new IntakeOut(this)); // Runnable to stop the intake motor when the command is interupted
  }

  public Command stop() {
    return runOnce(() -> intakeMotor.set(ControlMode.PercentOutput, 0)); // Runnable to stop the intake motor
  }

  // public Command inAutoStop(double speed/*, Timer t (should be passed, not created each call) */) {
  //   return new FunctionalCommand(
  //     () -> {
  //         intakeMotor.set(ControlMode.PercentOutput, speed);
  //     }, 
  //     () -> {
  //       // intakeMotor.set(ControlMode.PercentOutput, speed);
  //       //  Write control code here
  //       /*  I recommend this for ending the loop - Patrick
  //           if(colorSensor.getProximity() > 150 && !timer.hasElapsed(1)) {
  //             timer.start();
  //             intakeMotor.set(ControlMode.PercentOutput, -0.25);
  //           }
  //       */
  //     }, 
  //     (interrupted) -> {
  //       Timer timer = new Timer();
  //       timer.start();
  //       //BAD BAD BAD, gets whole robot stuck in loop
  //       while (colorSensor.getProximity() < 150 && timer.get() < 1) {
  //         intakeMotor.set(ControlMode.PercentOutput, -0.25);
  //       }
  //       // Only keep this line!
  //       intakeMotor.set(ControlMode.PercentOutput, 0);
  //     }, 
  //     () -> {
  //         // return t.hasElapsed(1);
  //         return colorSensor.getProximity() > 150;
  //     }, 
  //     this
  //   );
  // }

  public Command shoot(int speed) {
    return new FunctionalCommand(
        () -> {
            // Start the shooter motor at full speed
            shooterMotor.setControl(motionMagicVelocity.withVelocity(speed));
        },
        () -> {
            SmartDashboard.putNumber("shooter speed RPS", shooterMotor.getVelocity().getValueAsDouble());
            // Wait until the shooter motor is at full speed
            if (shooterMotor.getVelocity().getValue() >= (speed - 10)) {
                // Start the intake motor at full speed to feed the disc into the shooter
                intakeMotor.set(ControlMode.PercentOutput, 1);
            }
        },
        (interrupted) -> {
            // Stop both motors
            shooterMotor.setControl(motionMagicVelocity.withVelocity(0));
            intakeMotor.set(ControlMode.PercentOutput, 0);
        },
        () -> {
            // This command never finishes on its own, but it can be interrupted
            return false;
        },
        this // Subsytem requirement of the intake
    );
  }

  /** Starts the shooter and stops it when interuppted <p> Does not automatically feed the note like Intake.shoot() */
  public Command shootNoIntake(double speed) {
    return runEnd(() -> shooterMotor.setControl(motionMagicVelocity.withVelocity(speed)),
                  () -> shooterMotor.setControl(motionMagicVelocity.withVelocity(0)));
  }

  public Command startShooter(int speed) {
    return new FunctionalCommand(
        () -> {
            // Start the shooter motor at full speed
            shooterMotor.setControl(motionMagicVelocity.withVelocity(speed));
        },
        () -> {

        },
        (interrupted) -> {

        },
        () -> {
            return shooterMotor.getVelocity().getValue() >= (speed - 5); //FIXME changed this to 5 before comp thursday, was 10 when testing auton before but now shooter pids are tuned so it should work better this way
        },
        this // Subsytem requirement of the intake
    );
  }

  public Command shoot1Sec(int speed) {
    return new FunctionalCommand(
        () -> {
            // Start the shooter motor at full speed
            shooterMotor.setControl(motionMagicVelocity.withVelocity(speed));
        },
        () -> {
            SmartDashboard.putNumber("shooter speed RPS", shooterMotor.getVelocity().getValueAsDouble());
            // Wait until the shooter motor is at full speed
            if (shooterMotor.getVelocity().getValue() >= (speed - 5)) {
                // Start the intake motor at full speed to feed the disc into the shooter
                intakeMotor.set(ControlMode.PercentOutput, 1);
            }
        },
        (interrupted) -> {
            // Stop both motors
            shooterMotor.setControl(motionMagicVelocity.withVelocity(0));
            intakeMotor.set(ControlMode.PercentOutput, 0);
        },
        () -> {
          Timer timer = new Timer();
          timer.start();
          return timer.hasElapsed(1);
        },
        this // Subsytem requirement of the intake
    );
  }

  public Command setShooter(double speed) {
    return runOnce(() -> shooterMotor.setControl(motionMagicVelocity.withVelocity(speed)));
  }

  public Command stopShooter() {
    return runOnce(() -> shooterMotor.setControl(motionMagicVelocity.withVelocity(0)));
  }

  // public boolean checkColorSensor() {
  //   return colorSensor.getProximity() > 150;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("colorProx", colorSensor.getProximity());

    SmartDashboard.putNumber("intake stator current", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("intake supply current", intakeMotor.getSupplyCurrent());

    SmartDashboard.putNumber("shooter RPS", shooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInput(shooterMotor.getMotorVoltage().getValueAsDouble());
    flywheelSim.update(0.02);

    shooterSim.setRotorVelocity(flywheelSim.getAngularVelocityRPM() / 60);
  }
}

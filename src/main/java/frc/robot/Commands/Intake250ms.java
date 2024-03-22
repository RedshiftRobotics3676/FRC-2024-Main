// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.RobotConstants.intakeInSpeed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake250ms extends SequentialCommandGroup {
  Intake intake;
  /** Creates a new Intake250ms. */
  public Intake250ms(Intake intake) {
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(intake.set(intakeInSpeed));
    addCommands(new WaitCommand(0.25));
    addCommands(intake.stop());
  }
}

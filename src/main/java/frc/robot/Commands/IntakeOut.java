// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.RobotConstants.intakeOutSpeed;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeOut extends ParallelDeadlineGroup {
  Intake intake;
  /** Creates a new IntakeOut. */
  public IntakeOut(Intake intake) {
    super(new WaitCommand(0.25));
    this.intake = intake;
    // addRequirements(intake);
    
    addCommands(intake.out(intakeOutSpeed));
    addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("intake out", true)));
  }
}

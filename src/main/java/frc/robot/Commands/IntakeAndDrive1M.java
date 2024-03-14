// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndDrive1M extends ParallelDeadlineGroup {
  Intake intake;
  Arm arm;
  CommandSwerveDrivetrain drive;
  /** Creates a new IntakeIn. */
  public IntakeAndDrive1M(Intake intake, Arm arm, CommandSwerveDrivetrain drive) {
    super(new WaitCommand(1));
    this.intake = intake;
    this.arm = arm;
    this.drive = drive;
    // addRequirements(intake);
    // addRequirements(arm);
    // addRequirements(drive);
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(arm.setArmPositionEnd(0.00));
    addCommands(intake.in(0.6));
    addCommands(drive.driveRobotRelativeCommand(new ChassisSpeeds(1, 0, 0)));
    // addCommands(new SequentialCommandGroup(new WaitCommand(0.9), arm.setArmPosition(0.035)));
    // addCommands(new SequentialCommandGroup(new WaitCommand(0.9), intake.set(0)));
  }
}

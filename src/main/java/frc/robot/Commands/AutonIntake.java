// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.RobotConstants.armDownPos;
import static frc.robot.Constants.RobotConstants.intakeInSpeed;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntake extends ParallelCommandGroup {
  Intake intake;
  Arm arm;
  /** Creates a new AutonIntake. */
  public AutonIntake(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;
    
    addCommands(arm.setArmPositionEnd(armDownPos));
    addCommands(intake.inThenOut(intakeInSpeed));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.RobotConstants.armDefaultPos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BeginAuton extends SequentialCommandGroup {
  Intake intake;
  Arm arm;
  /** Creates a new BeginAuton. */
  public BeginAuton(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;

    // addCommands(new ParallelDeadlineGroup(
    //                 new WaitCommand(1), 
    //                 arm.setArmPositionOnce(0.035), 
    //                 intake.setShooter(65)));
    addCommands(intake.setShooter(55));
    addCommands(arm.setArmPositionCheck(armDefaultPos));
    addCommands(new AutonShoot(intake));

    // addCommands(new SequentialCommandGroup(
    //                 intake.set(1), 
    //                 new WaitCommand(0.25), 
    //                 intake.stop(), 
    //                 intake.setShooter(0)));
  }
}

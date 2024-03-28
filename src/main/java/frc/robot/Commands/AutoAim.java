// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Subsystems.Arm;
// import frc.robot.Subsystems.CommandSwerveDrivetrain;
// import frc.robot.Subsystems.Vision;

// public class AutoAim extends Command {
//   private final CommandSwerveDrivetrain drivetrain;
//   // private final Arm arm;
//   private final Vision vision;

//   private final CommandXboxController driver;
//   private final double maxSpeed;
//   private final double maxAngularRate;
//   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(0.05).withRotationalDeadband(0.05);
//   /** Creates a new AutoAim. */
//   public AutoAim(CommandSwerveDrivetrain drivetrain, Arm arm, Vision vision, CommandXboxController driver, double maxSpeed, double maxAngularRate) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drivetrain = drivetrain;
//     // this.arm = arm;
//     this.vision = vision;
//     this.driver = driver;
//     this.maxSpeed = maxSpeed;
//     this.maxAngularRate = maxAngularRate;
//     addRequirements(drivetrain);
//     addRequirements(arm);
//     addRequirements(vision);
//   }

//   private double cubeInputs(double input) {
//     return Math.abs(Math.pow(input, 3)) * Math.signum(input);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Rotate the robot towards the speaker while still allowing for driver translation control
//     drivetrain.applyRequest(() -> drive.withVelocityX(cubeInputs(-driver.getLeftY()) * maxSpeed)
//             .withVelocityY(cubeInputs(-driver.getLeftX()) * maxSpeed)
//             .withRotationalRate(vision.calcRotationSpeed(vision.getPipelineResult(), cubeInputs(-driver.getRightX()) * maxAngularRate))
//       );

    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

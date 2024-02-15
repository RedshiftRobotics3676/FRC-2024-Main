// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Music;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  /**
   * Desired top speed in meters per second
   * 
   * <p> The default value is 6 meters per second
   * 
   * <p> The physical max speed of the MK4i L3 modules is 18.2 feet per second
   *     which is aproximately 5.54736 meters per second
   */
  final double MaxSpeed = 5.54736; //defualt 6 meters per second desired top speed
  /**
   * Desired top rotation speed in radians per second
   * 
   * <p> The default value is pi radians per second
   */
  final double MaxAngularRate = 2*Math.PI; //default - pi = Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
  TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight); // My drivetrain
  // SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true/* default - true */); // I want field-centric
  //                                                                                           // driving in open loop
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(0.07);
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt().withIsOpenLoop(false);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  Limelight limelight = new Limelight();
  Music music = new Music(drivetrain);
  LEDs leds = new LEDs();
  Arm arm = new Arm();
  // Intake intake = new Intake();

  SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
  SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  // Build an auto chooser. This will use Commands.none() as the default option.
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("3m forward auto");
  private final SendableChooser<Double> armPosChooser = new SendableChooser<Double>();

  private double squareInputs(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(squareInputs(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(squareInputs(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(squareInputs(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // arm.setDefaultCommand(arm.setArmSpeed(joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain // Not running continuously correctly? only when button is pressed ocne not held
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.y().onTrue(music.loadFromChooser());
    joystick.x().onTrue(music.playOrPause());

    // // joystick.rightTrigger(.2).whileTrue(arm.setArmSpeed(joystick.getRightTriggerAxis()));
    // joystick.start().whileTrue(intake.shoot());

    joystick.povUp().onTrue(arm.positionDutyCycle(0.05));
    joystick.povLeft().onTrue(arm.positionDutyCycle(0.025));
    joystick.povRight().onTrue(arm.positionDutyCycle(0.01));
    joystick.povDown().onTrue(arm.positionDutyCycle(0.00));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      // SmartDashboard.putString("states", Arrays.toString(drivetrain.getState().ModuleStates));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // armPosChooser.setDefaultOption("0.00", 0.0);

    // armPosChooser.addOption("0.00", 0.0);
    // armPosChooser.addOption("0.01", 0.01);
    // armPosChooser.addOption("0.02", 0.02);
    // armPosChooser.addOption("0.03", 0.03);
    // armPosChooser.addOption("0.04", 0.04);

    // arm.setDefaultCommand(new RunCommand(() -> arm.setArmPosition(armPosChooser.getSelected())));
  }

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // SmartDashboard.putNumber(null, MaxAngularRate);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

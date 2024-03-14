// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutoAim;
import frc.robot.Commands.IntakeAndDrive1M;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.Shoot2;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Music;
import frc.robot.Subsystems.Vision;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  /**
   * Desired top speed in meters per second
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
  /** Subsystem opperator controller <p><b> USB Port 1 */
  CommandXboxController secondary = new CommandXboxController(1);

  CommandXboxController guitar = new CommandXboxController(2);

  CommandJoystick joystick = new CommandJoystick(3);

  // CommandJoystick flightStick = new CommandJoystick(3);

  /** Driver controller <p><b> USB Port 0 */
  CommandXboxController driver = new CommandXboxController(0);
  public CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
  TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight); // My drivetrain
  // SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true/* default - true */); // I want field-centric
  //                                                                                           // driving in open loop
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(0.05).withRotationalDeadband(0.05).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt().withIsOpenLoop(false);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  // Vision vision = new Vision();
  Music music = new Music(drivetrain);
  LEDs leds = new LEDs();
  Arm arm = new Arm();
  Intake intake = new Intake();
  Elevator elevator = new Elevator();

  
  SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
  SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  private final SendableChooser<Command> autoChooser;

  private double cubeInputs(double input) {
    return Math.abs(Math.pow(input, 3)) * Math.signum(input);
  }

  // private double mapAndCubeInputs(double input, double inMin, double inMax, double outMin, double outMax) {
  //   return (input - inMin) * (outMax - outMin) * (inMax - inMin) + outMin;
  // // }

  // private double deadband = 0.05;
  // private double mapAndCubeInputs(double input) {
  //   return (Math.abs(input) - deadband) * MaxSpeed * (1 - deadband);
  // }

  private double getThrottle() {
    return (-joystick.getRawAxis(2) + 1) / 2;
  }

  private void configureBindings() {
    drivetrain.seedFieldRelative();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(cubeInputs(-driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(cubeInputs(-driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(cubeInputs(-driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(cubeInputs(-joystick.getRawAxis(1)) * getThrottle() * MaxSpeed) // Drive forward with negative Y (forward)
    //         .withVelocityY(cubeInputs(-joystick.getRawAxis(0)) * getThrottle() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(cubeInputs(-joystick.getRawAxis(3)) * getThrottle() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    // ));

    //auto aim towards nearest apriltag
    // if (driver.getHID().getAButton()) {
    //   drivetrain.applyRequest(() -> drive.withVelocityX(cubeInputs(-driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
    //         .withVelocityY(cubeInputs(-driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(vision.calcRotationSpeed((cubeInputs(-driver.getRightX()) * MaxAngularRate))) // Drive counterclockwise with negative X (left)
    //   );
    // }

    // // Moved to teleopPeriodic()
    // arm.setDefaultCommand(arm.setArmPosition(0.035));

    // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-secondary.getLeftY(), -secondary.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // secondary.y().onTrue(music.loadFromChooser());
    // secondary.x().onTrue(music.playOrPause());

    // driver.a().whileTrue(intake.inAutoStop(0.6));
    driver.a().whileTrue(intake.in(0.6));
    driver.b().whileTrue(intake.out(0.25));

    driver.x().whileTrue(intake.shoot(45));
    driver.y().whileTrue(intake.shoot(35));

    // Trigger colorSensorDetect = new Trigger(intake::checkColorSensor);
    // colorSensorDetect.onTrue(new RunCommand(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.5)));

    // secondary.a().and(colorSensorDetect).onTrue(new RunCommand(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.5)));;


    driver.povUp().whileTrue(arm.setArmPosition(0.255));
    driver.povLeft().whileTrue(arm.setArmPosition(0.075));
    driver.povRight().whileTrue(arm.setArmPosition(0.035));
    driver.povDown().whileTrue(arm.setArmPosition(0.00));

    // driver.rightBumper().whileTrue(new AutoAim(drivetrain, arm, vision, driver, MaxSpeed, MaxAngularRate));

    // driver.leftTrigger(0.05).whileTrue(arm.incrementArmPosition(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()));
    // driver.rightTrigger(0.05).whileTrue(arm.incrementArmPosition(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()));

    // driver.rightTrigger(0.05).whileTrue(arm.incrementArmPosition(driver.getRightTriggerAxis()));

    //---------------------------------------------------------------------------------------------------------------------------------

    // secondary.a().whileTrue(intake.inAutoStop(0.6));
    secondary.a().whileTrue(intake.in(0.6));
    secondary.b().whileTrue(intake.out(0.25));

    secondary.x().whileTrue(intake.shoot(65));
    // secondary.y().whileTrue(intake.shoot(75));
    secondary.y().whileTrue(new Shoot(intake));

    secondary.rightBumper().whileTrue(intake.in(1));
    secondary.leftBumper().whileTrue(intake.shoot(30));

    // arm.setDefaultCommand(arm.setArmPosition(0.035));

    secondary.povUp().whileTrue(arm.setArmPosition(0.255));
    secondary.povLeft().whileTrue(arm.setArmPosition(0.075));
    secondary.povRight().whileTrue(arm.setArmPosition(0.035));
    secondary.povDown().whileTrue(arm.setArmPosition(0.00));

    secondary.start().whileTrue(arm.setArmPosition(0.225));

    // secondary.axisGreaterThan(2, 0.05).whileTrue(elevator.downOutput(secondary.getLeftTriggerAxis())).onFalse(elevator.stop());
    // secondary.axisGreaterThan(3, 0.05).whileTrue(elevator.upOutput(secondary.getRightTriggerAxis())).onFalse(elevator.stop());

    secondary.axisGreaterThan(2, 0.05).whileTrue(new RunCommand(() -> elevator.down2(secondary.getLeftTriggerAxis()), elevator).repeatedly()).onFalse(new RunCommand(() -> elevator.stop2(), elevator));
    secondary.axisGreaterThan(3, 0.05).whileTrue(new RunCommand(() -> elevator.up2(secondary.getRightTriggerAxis()), elevator).repeatedly()).onFalse(new RunCommand(() -> elevator.stop2(), elevator));

    // secondary.leftTrigger(0.05).whileTrue(arm.incrementArmPosition(secondary.getRightTriggerAxis() - secondary.getLeftTriggerAxis()));
    // secondary.rightTrigger(0.05).whileTrue(arm.incrementArmPosition(secondary.getRightTriggerAxis() - secondary.getLeftTriggerAxis()));
    // secondary.leftTrigger(0.05).whileTrue(arm.incrementArmPosition(-secondary.getLeftTriggerAxis()));
    // secondary.rightTrigger(0.05).whileTrue(arm.incrementArmPosition(secondary.getRightTriggerAxis()));

    //---------------------------------------------------------------------------------------------------------------------------------

    joystick.button(2).whileTrue(intake.in(0.75));
    joystick.button(4).whileTrue(intake.out(0.3));

    joystick.button(1).whileTrue(intake.shoot(45));

    joystick.button(5).whileTrue(arm.setArmPosition(0.245));
    joystick.button(6).whileTrue(arm.setArmPosition(0.85));
    joystick.button(9).whileTrue(arm.setArmPosition(0.00));

    //---------------------------------------------------------------------------------------------------------------------------------


    // guitar.a().onTrue(intake.inAutoStop(0.75));
    guitar.a().onTrue(intake.in(0.75));
    guitar.b().onTrue(intake.out(0.3));

    guitar.povDown().whileTrue(intake.shoot(45));
    
    guitar.povUp().onTrue(arm.setArmPosition(0.245));
    guitar.povRight().onTrue(arm.setArmPosition(0.085));
    guitar.povLeft().onTrue(arm.setArmPosition(0.035));

    //No idea why this doesn't work - The value passed into the setArmPositionAverage method is always 0 no matter what the controller axis value is
    // guitar.start().whileTrue(arm.setArmPositionAverage((Math.abs(guitar.getRawAxis(5)) / 4.0)).repeatedly());
    // guitar.button(7).whileTrue(Commands.print(""+guitar.getRightY()));
    // guitar.start().whileTrue(arm.setArmPositionAverage(guitar.getRawAxis(5)));
    // guitar.start().whileTrue(new RunCommand(() -> {SmartDashboard.putNumber("axis5", (Math.abs(guitar.getRawAxis(5)) / 4.0));}).repeatedly());


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      // SmartDashboard.putString("states", Arrays.toString(drivetrain.getState().ModuleStates));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("intake start", intake.set(0.75));
    NamedCommands.registerCommand("intake start out", intake.set(-0.3));
    NamedCommands.registerCommand("intake stop", intake.set(0.00));
    NamedCommands.registerCommand("intake in", intake.in(0.75));
    NamedCommands.registerCommand("shooter start", intake.setShooter(50));
    NamedCommands.registerCommand("shooter stop", intake.setShooter(0));
    NamedCommands.registerCommand("arm down", arm.setArmPositionOnce(0.00));
    NamedCommands.registerCommand("arm mid", arm.setArmPositionOnce(0.035));
    NamedCommands.registerCommand("arm high", arm.setArmPositionOnce(0.255));
    NamedCommands.registerCommand("drive forward", drivetrain.driveRobotRelativeCommand(new ChassisSpeeds(1, 0, 0)));
    NamedCommands.registerCommand("drive stop", drivetrain.driveRobotRelativeCommand(new ChassisSpeeds(0, 0, 0)));
    
    NamedCommands.registerCommand("shoot", new Shoot(intake));
    NamedCommands.registerCommand("intake 1M", new IntakeAndDrive1M(intake, arm, drivetrain));
    NamedCommands.registerCommand("intake out", new IntakeOut(intake));
    // NamedCommands.registerCommand("drive forward", new RunCommand(() -> robotCentric.withVelocityX(1), drivetrain));
    // NamedCommands.registerCommand("drive stop", new RunCommand(() -> robotCentric.withVelocityX(0), drivetrain));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

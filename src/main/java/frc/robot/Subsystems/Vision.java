// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Vision extends SubsystemBase {
  private PhotonCamera driverCam;

  private PhotonCamera apriltagCam;
  private AprilTagFieldLayout aprilTagFieldLayout;
  
  //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Transform3d robotToCam;

  PhotonPoseEstimator photonPoseEstimator;

  // Constants such as camera and target height stored.
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(14.25);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5); //FIXME find speaker tag height
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    
  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D, RobotConstants.LOOP_PERIOD_SECONDS);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D, RobotConstants.LOOP_PERIOD_SECONDS);  

  /** Creates a new Vision. */
  public Vision() {
    driverCam = new PhotonCamera("lifecam");

    apriltagCam = new PhotonCamera("arducam");

    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-3/4), Units.inchesToMeters(-1), Units.inchesToMeters(14.25)), new Rotation3d(0, 0, Units.degreesToRadians(175)+0.265));

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, apriltagCam, robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
    return pose;
    // if (pose.isPresent())
    // else
  }

  public PhotonPipelineResult getPipelineResult() {
    return apriltagCam.getLatestResult();
  }

  public double calcRotationSpeed(PhotonPipelineResult result, double defaultSpeed) {
    double rotationSpeed = defaultSpeed;
    if (result.hasTargets()) {
      int id = result.getBestTarget().getFiducialId();
      if (id == 4 || id == 7) {
        rotationSpeed = -turnController.calculate(Units.degreesToRadians(result.getBestTarget().getYaw()), 0);
      }
    }
    return rotationSpeed;
  }

  // public double calcArmPosition(PhotonPipelineResult result) {
  //   double defaultPos = 0.035;
  //   double distance = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

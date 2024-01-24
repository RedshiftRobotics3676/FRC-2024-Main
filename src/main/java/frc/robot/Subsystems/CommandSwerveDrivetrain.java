package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    StructArrayPublisher<SwerveModuleState> publisher;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

        BooleanSupplier flipped = () -> false;
        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        flipped, // 
        this // Reference to this subsystem to set requirements
        );
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

        BooleanSupplier flipped = () -> false;
        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 1.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 1.0), // Rotation PID constants
            5.5, // Max module speed, in m/s
            0.381, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        flipped, // 
        this // Reference to this subsystem to set requirements
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        pose = new Pose2d();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(m_kinematics.toChassisSpeeds(getState().ModuleStates), getRotation3d().toRotation2d());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelative) {
        applyRequest(() -> new SwerveRequest.ApplyChassisSpeeds().withSpeeds(robotRelative));
    }

    @Override
    public void periodic() {
        publisher.set(getState().ModuleStates);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }
}

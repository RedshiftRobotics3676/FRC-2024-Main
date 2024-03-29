package frc.robot.Subsystems;

import frc.robot.Constants.RobotConstants;
import frc.robot.generated.TunerConstants;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

// import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    StructArrayPublisher<SwerveModuleState> publisher;
    SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
    SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    // private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        // for (SwerveModule module : super.Modules) {
        //     module.getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        // }

        Modules[0].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kFrontLeftEncoderOffset));
        Modules[1].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kFrontRightEncoderOffset));
        Modules[2].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kBackLeftEncoderOffset));
        Modules[3].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kBackRightEncoderOffset));
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        // for (SwerveModule module : super.Modules) {
        //     module.getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        // }

        Modules[0].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kFrontLeftEncoderOffset));
        Modules[1].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kFrontRightEncoderOffset));
        Modules[2].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kBackLeftEncoderOffset));
        Modules[3].getCANcoder().getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-TunerConstants.kBackRightEncoderOffset));

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


        // Optional<Alliance> ally = DriverStation.getAlliance();
        BooleanSupplier flipped = () -> {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                return ally.get() == Alliance.Red;
            }
            else {
                return false;
            }
        };

        if (Utils.isSimulation()) {
            for (SwerveModule module : Modules) {
                module.getDriveMotor().getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(300));
            }
        }

        SmartDashboard.putBoolean("auton flipped for red", flipped.getAsBoolean());

        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative1, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(7.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(2.5, 0.0, 0.25), // Rotation PID constants
            5.55, // Max module speed, in m/s
            0.377, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
            //RobotConstants.LOOP_PERIOD_SECONDS
        ),
        flipped, // true for red side, false for blue side
        this // Reference to this subsystem to set requirements
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d getPose() {
        return super.m_odometry.getEstimatedPosition();
        // return getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        super.m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(m_kinematics.toChassisSpeeds(getState().ModuleStates[0], getState().ModuleStates[1], getState().ModuleStates[2], getState().ModuleStates[3]), getPigeon2().getRotation2d());
    }

    /* public void driveRobotRelative(ChassisSpeeds robotRelative) {
        // applyRequest(() -> applyChassisSpeeds.withSpeeds(robotRelative).withDriveRequestType(DriveRequestType.Velocity));
        applyRequest(() -> robotCentric.withVelocityX(robotRelative.vxMetersPerSecond)
                                       .withVelocityY(robotRelative.vyMetersPerSecond)
                                       .withRotationalRate(robotRelative.omegaRadiansPerSecond)).schedule();
        
        SmartDashboard.putString("getChassisSpeeds", getRobotRelativeSpeeds().toString());
        SmartDashboard.putString("driveChassisSpeeds", robotRelative.toString());
    } */

    /** Drive the robot relative to the field with the speeds from the given ChassisSpeeds */
    public void driveRobotRelative1(ChassisSpeeds robotRelative) {
        // applyRequest(() -> applyChassisSpeeds.withSpeeds(robotRelative).withDriveRequestType(DriveRequestType.Velocity));
        this.setControl(robotCentric.withVelocityX(robotRelative.vxMetersPerSecond)
                               .withVelocityY(robotRelative.vyMetersPerSecond)
                               .withRotationalRate(robotRelative.omegaRadiansPerSecond));
        // SmartDashboard.putString("getChassisSpeeds", getRobotRelativeSpeeds().toString());
        // SmartDashboard.putString("driveChassisSpeeds", robotRelative.toString());
    }

    public Command driveRobotRelativeCommand(ChassisSpeeds robotRelative) {
        return runEnd(() -> this.setControl(robotCentric.withVelocityX(robotRelative.vxMetersPerSecond)
                                .withVelocityY(robotRelative.vyMetersPerSecond)
                                .withRotationalRate(robotRelative.omegaRadiansPerSecond)),

                      () -> this.setControl(robotCentric.withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0)));
    }

    /* public void driveRobotRelative2(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] moduleStates = super.m_simDrive.Kinem.toSwerveModuleStates(targetSpeeds);
        // setControl(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
        super.getModule(0).apply(moduleStates[0], DriveRequestType.OpenLoopVoltage);
        super.getModule(1).apply(moduleStates[1], DriveRequestType.OpenLoopVoltage);
        super.getModule(2).apply(moduleStates[2], DriveRequestType.OpenLoopVoltage);
        super.getModule(3).apply(moduleStates[3], DriveRequestType.OpenLoopVoltage);
        // super.m_simDrive.
    } */
    

    // Assuming this is a method in your drive subsystem
    /* public Command followPathCommand(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        BooleanSupplier flipped = () -> false;

        // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
        return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative1, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(.5, 0.0, 0.8), // Translation PID constants
                new PIDConstants(.5, 0.0, 0.8), // Rotation PID constants
                5.54, // Max module speed, in m/s
                0.381, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            flipped,
            this // Reference to this subsystem to set requirements
        );
    } */

    // public void setEstimatedPose(Optional<EstimatedRobotPose> estimatedPose) {
    //     this.estimatedPose = estimatedPose;
    // }

    @Override
    public void periodic() {
        publisher.set(getState().ModuleStates);
        SmartDashboard.putNumber("FL Angle", Modules[0].getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("FR Angle", Modules[1].getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BL Angle", Modules[2].getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BR Angle", Modules[3].getCANcoder().getAbsolutePosition().getValueAsDouble());

        // if (estimatedPose.isPresent()) {
        //     // if (estimatedPose.get().estimatedPose.toPose2d().relativeTo()) {}
        //     addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds); 
        // }
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(RobotConstants.LOOP_PERIOD_SECONDS, RobotController.getBatteryVoltage());
    }
}

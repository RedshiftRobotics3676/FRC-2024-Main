package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
// import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot
    // The steer motor uses MotionMagicVoltage control
    public static final Slot0Configs steerGains = new Slot0Configs()
        /* .withKP(0.3).withKI(0).withKD(0.5)
        .withKS(1).withKV(0).withKA(0); */
        //.withKP(.2).withKI(0).withKD(.1)
        //.withKS(0).withKV(0).withKA(0); 
        .withKP(-75).withKI(0).withKD(-1)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses:
    // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
    // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
    public static final Slot0Configs driveGains = new Slot0Configs() //TODO find drive pids maybe?
        //.withKP(3).withKI(0).withKD(0)
        //.withKS(0).withKV(0).withKA(0);
        .withKP(0.2).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 6.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.5714285714285716;

    public static final double kDriveGearRatio = 6.122448979591837;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final double kWheelRadiusInches = 2;

    public static final boolean kSteerMotorReversed = false;
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final String kCANbusName = "";
    public static final int kPigeonId = 1;


    // These are only used for simulation
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 1;
    public static final int kFrontLeftEncoderId = 21;
    public static final double kFrontLeftEncoderOffset = 0.302490234375;

    public static final double kFrontLeftXPosInches = 10;
    public static final double kFrontLeftYPosInches = 10;

    // Front Right
    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 3;
    public static final int kFrontRightEncoderId = 22;
    public static final double kFrontRightEncoderOffset = 0.3154296875;

    public static final double kFrontRightXPosInches = 10;
    public static final double kFrontRightYPosInches = -10;

    // Back Left
    public static final int kBackLeftDriveMotorId = 6;
    public static final int kBackLeftSteerMotorId = 5;
    public static final int kBackLeftEncoderId = 23;
    public static final double kBackLeftEncoderOffset = 0.228759765625;

    public static final double kBackLeftXPosInches = -10;
    public static final double kBackLeftYPosInches = 10;

    // Back Right
    public static final int kBackRightDriveMotorId = 8;
    public static final int kBackRightSteerMotorId = 7;
    public static final int kBackRightEncoderId = 24;
    public static final double kBackRightEncoderOffset = -0.484619140625;

    public static final double kBackRightXPosInches = -10;
    public static final double kBackRightYPosInches = -10;


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    // Moved drivetrain initialization to RobotContainer
    /* public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight); */
}

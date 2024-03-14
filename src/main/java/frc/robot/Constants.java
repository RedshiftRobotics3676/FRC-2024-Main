package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {
    public static class RobotConstants {
        public static final double LOOP_PERIOD_SECONDS = 0.002; 
    }

    public static class IntakeConstants {
        public static final int kIntakeMotorID = 31;
        public static final int kShooterMotorID = 32;

        private static final MotorOutputConfigs kShooterOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        
        private static final Slot0Configs kShooterGains = new Slot0Configs()
            .withKP(0.375).withKI(0.0).withKD(0.0)
            .withKS(3.25).withKV(0.0).withKA(0.0);

        private static final MotionMagicConfigs kShooterMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(100)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);

        public static final TalonFXConfiguration kShooterConfigs = new TalonFXConfiguration()
            .withMotorOutput(kShooterOutputConfigs)
            .withSlot0(kShooterGains)
            .withMotionMagic(kShooterMotionMagicConfigs);

        
        public static final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0.5)
            .withOverrideBrakeDurNeutral(true)
            .withEnableFOC(false)
            .withSlot(0);
    }

    public static class ArmConstants {
        public static final int kArmLeaderPort = 41; // Left
        public static final int kArmFollowerPort = 42; // Right
        public static final int kArmCanCoderPort = 43;

        private static final MotorOutputConfigs kArmOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // was CounterClockwise_Positive
            .withNeutralMode(NeutralModeValue.Brake);
        
        private static final Slot0Configs kArmGains = new Slot0Configs()
            .withKP(75).withKI(0.0).withKD(0.0)
            .withKS(0.0).withKV(0.0).withKA(0.0)
            .withKG(0.12).withGravityType(GravityTypeValue.Arm_Cosine);

        private static final FeedbackConfigs kArmFeedback = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(kArmCanCoderPort)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withRotorToSensorRatio(174.2222222222222) // Calculate by dividing big gear by small gear then multiply by 50 for the gearbox (64/18 * 50)
            .withSensorToMechanismRatio(1);

        private static final SoftwareLimitSwitchConfigs kArmSoftLimitSwitch = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.255)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

        private static final MotionMagicConfigs kArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(1.75)
            .withMotionMagicCruiseVelocity(2.25)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);

        public static final TalonFXConfiguration kArmConfigs = new TalonFXConfiguration()
            .withMotorOutput(kArmOutputConfigs)
            .withSlot0(kArmGains)
            .withFeedback(kArmFeedback)
            .withSoftwareLimitSwitch(kArmSoftLimitSwitch)
            .withMotionMagic(kArmMotionMagicConfigs);


        private static final MagnetSensorConfigs kMagnetSensorConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(-0.39453125)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive) // was CounterClockwise_Positive
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);

        public static final CANcoderConfiguration kArmCANCoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(kMagnetSensorConfigs);



        public static final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0)
            .withOverrideBrakeDurNeutral(true)
            .withEnableFOC(false)
            .withSlot(0);
            
        public static final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0.5)
            .withOverrideBrakeDurNeutral(true)
            .withEnableFOC(false)
            .withSlot(1);
    }

    public static class ElevatorConstants {
        public static final int kElevatorPort = 51;

        private static final MotorOutputConfigs kElevatorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        
        private static final Slot0Configs kElevatorGains = new Slot0Configs()
            .withKP(0).withKI(0.0).withKD(0.0)
            .withKS(0.0).withKV(0.0).withKA(0.0)
            .withKG(0.0).withGravityType(GravityTypeValue.Elevator_Static);

        private static final SoftwareLimitSwitchConfigs kElevatorSoftLimitSwitch = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(92)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

        private static final MotionMagicConfigs kElevatorMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(1)
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);

        public static final TalonFXConfiguration kArmConfigs = new TalonFXConfiguration()
            .withMotorOutput(kElevatorOutputConfigs)
            .withSlot0(kElevatorGains)
            .withSoftwareLimitSwitch(kElevatorSoftLimitSwitch)
            .withMotionMagic(kElevatorMotionMagicConfigs);

        public static final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0)
            .withOverrideBrakeDurNeutral(true)
            .withEnableFOC(false)
            .withSlot(0);
    }

    public static class LEDConstants {
        public static final int kPwmPort = 0;
        public static final int kLedLength = 300;
        public static final int kRainbowFirstPixelHue = 0;
        public static final int kAllianceColor = 0;
    }
}
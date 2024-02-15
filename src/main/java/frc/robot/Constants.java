package frc.robot;

import javax.swing.SortingFocusTraversalPolicy;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {

    public static class IntakeConstants {
        public static final int kIntakeMotorPort = 31;
        public static final int kShooterMotorPort = 32;
    }

    public static class ArmConstants {
        public static final int kArmLeaderPort = 42;
        public static final int kArmFollowerPort = 41;
        public static final int kArmCanCoderPort = 43;

        private static final MotorOutputConfigs kArmOutpudConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        
        private static final Slot0Configs kArmGains = new Slot0Configs()
            .withKP(0).withKI(0.0).withKD(0.0)
            .withKS(0.0).withKV(0.0).withKA(0.0)
            .withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine);

        private static final FeedbackConfigs kArmFeedback = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(kArmCanCoderPort)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

        private static final SoftwareLimitSwitchConfigs kArmSoftLimitSwitch = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.00)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0.00);

        public static final TalonFXConfiguration kArmConfig = new TalonFXConfiguration()
            .withMotorOutput(kArmOutpudConfigs)
            .withSlot0(kArmGains)
            .withFeedback(kArmFeedback)
            .withSoftwareLimitSwitch(kArmSoftLimitSwitch);
    }

    public static class LEDConstants {
        public static final int kPwmPort = 0;
        public static final int kLedLength = 300;
        public static final int kRainbowFirstPixelHue = 0;
        public static final int kAllianceColor = 0;
    }
}
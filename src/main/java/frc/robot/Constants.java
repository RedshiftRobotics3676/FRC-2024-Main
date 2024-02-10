package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class Constants {
    public static class IntakeConstants {
        public static final int kIntakeMotorPort = 31;
        public static final int kShooterMotorPort = 32;
    }
    public static class ArmConstants {
        public static final int kArmLeaderPort = 41;
        public static final int kArmFollowerPort = 42;
        public static final int kArmCanCoderPort = 43;

        
        private static final Slot0Configs kArmGains = new Slot0Configs()
        .withKP(0.2).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.0).withKA(0.0)
        .withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine);

        private static final FeedbackConfigs kArmFeedback = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(kArmCanCoderPort)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

        public static final TalonFXConfiguration kArmConfig = new TalonFXConfiguration()
            .withSlot0(kArmGains)
            .withFeedback(kArmFeedback);
    }
    public static class LEDConstants {
        public static final int kPwmPort = 0;
        public static final int kLedLength = 300;
        public static final int kRainbowFirstPixelHue = 0;
        public static final int kAllianceColor = 0;
    }
}
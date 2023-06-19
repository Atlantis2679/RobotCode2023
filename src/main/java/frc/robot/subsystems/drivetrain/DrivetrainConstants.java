package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class DrivetrainConstants {
    public static final int LEFT_ID = 4;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 6;

    public static final int LEFT_ENCODER_CHANNEL_A = 1;
    public static final int LEFT_ENCODER_CHANNEL_B = 0;
    public static final int RIGHT_ENCODER_CHANNEL_A = 4;
    public static final int RIGHT_ENCODER_CHANNEL_B = 5;

    public static final int CURRENT_LIMIT_AMP = 30;

    public static final double MAX_VELOCITY = 4.6; // meters per second
    public static final double PITCH_OFFSET = 0;

    public static final class ArcadeDrive {
        public static final double FORWARD_MULTIPLIER = 1;
        public static final double SENSITIVE_FORWARD_MULTIPLIER = 0.5;

        public static final double ROTATION_MULTIPLIER = 0.8;
        public static final double SENSITIVE_ROTATION_MULTIPLIER = 0.4;
    }

    public static final class DriveToDistance {
        public static final double KP = 1.3;
        public static final double KI = 0.005;
        public static final double KD = 0.3;

        public static final double POSITION_TOLERANCE = 0.07;
        public static final double VELOCITY_TOLERANCE = 0.02;
    }

    public static final class TurnByAngle {
        public static final double KP = 1.27;
        public static final double KI = 0;
        public static final double KD = 0.3;

        public static final double POSITION_TOLERANCE = 0.05;
        public static final double VELOCITY_TOLERANCE = 0.02;
    }

    public static final class AutoPath {
        public static final double KS = 1.0573;
        public static final double KV = 0.92391;
        public static final double KA = 0.883;

        public static final double WHEEL_DISTANCE = 0.45;

        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_VELOCITY = 1;

        public static final double K_RAMESTE_B = 1;
        public static final double K_RAMESTE_ZETA = 2;

        public static final double KP = 0;
    }
}

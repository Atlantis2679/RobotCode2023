package frc.robot.subsystems.drivetrain;

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
    public static final double PITCH_OFFSET_DEFAULT = 0;

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

    public static final class startPos {
        public static final double pos1x = 1.95; public static final double pos1y = 4.96;
        public static final double pos2x = 1.95; public static final double pos2y = 4.42;
        public static final double pos3x = 1.95; public static final double pos3y = 3.84;
        public static final double pos4x = 1.95; public static final double pos4y = 3.3;
        public static final double pos5x = 1.95; public static final double pos5y = 2.76;
        public static final double pos6x = 1.95; public static final double pos6y = 2.18;
        public static final double pos7x = 1.95; public static final double pos7y = 1.61;
        public static final double pos8x = 1.95; public static final double pos8y = 1.08;
        public static final double pos9x = 1.95; public static final double pos9y = 0.45;
        public static final double defaultX = 8.3; public static final double defaultY = 4;
        public static final double distanceFromSides = 12.68;
    }
}

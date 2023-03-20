package frc.robot.subsystems.arm;

public final class ArmConstants {
  public static final int MOTOR_SHOULDER_ID = 9;
  public static final int MOTOR_SHOULDER_FOLLOWER_ID = 10;
  public static final int MOTOR_ELBOW_ID = 11;
  public static final int ENCODER_SHOULDER_ID = 6;
  public static final int ENCODER_ELBOW_ID = 7;
  public static final int LIMIT_SWITCH_ID = 10;
  public static final double ENCODER_SHOULDER_ZERO_ANGLE = -48.477;
  public static final double ENCODER_ELBOW_ZERO_ANGLE = 333.678;
  public static final double ENCODER_MAX_POSITIVE_SHOULDER = 150;
  public static final double ENCODER_MAX_POSITIVE_ELBOW = 200;

  public static final int CURRENT_LIMIT_SHOULDER_AMP = 20;
  public static final int CURRENT_LIMIT_ELBOW_AMP = 20;

  public static final double SPEED_LIMIT_SHOULDER = 0.8;
  public static final double SPEED_LIMIT_ELBOW = 0.7;

  public static final double ANGLE_SECOND_CONE_SHOULDER = -20, ANGLE_SECOND_CONE_ELBOW = 50;
  public static final double ANGLE_THIRD_CONE_SHOULDER = 5, ANGLE_THIRD_CONE_ELBOW = 25;
  public static final double ANGLE_SECOND_CUBE_SHOULDER = -30, ANGLE_SECOND_CUBE_ELBOW = 55;
  public static final double ANGLE_THIRD_CUBE_SHOULDER = 0, ANGLE_THIRD_CUBE_ELBOW = 45;
  public static final double ANGLE_REST_SHOULDER = -85, ANGLE_REST_ELBOW = 155;
  public static final double ANGLE_FEEDER_SHOULDER = 5, ANGLE_FEEDER_ELBOW = 0;
  public static final double ANGLE_FLOOR_SHOULDER = -35, ANGLE_FLOOR_ELBOW = -55;

  public static final double ANGLE_TOUCH_AND_GO_SHOULDER = -70, ANGLE_TOUCH_AND_GO_ELBOW = 65;

  public static final double LOCKED_MAX_SHOULDER_ANGLE = -75;
  public static final double LOCKED_MIN_ELBOW_ANGLE = 115;

  public static final class Feedforward {
    public static final class Shoulder {
      public static final double KP = 0.01;
      public static final double KI = 0.0;
      public static final double KD = 0.0;

      public static final double KV = 0.18;
      public static final double KG = 0.25;
      public static final double KS = 0;
      public static final double KA = 0.0;

      public static final double TOLERANCE_POSITION = 10;
      public static final double TOLERANCE_VELOCITY = 2;
      public static final double MAX_VELOCITY = 45;
      public static final double MAX_ACCELERATION = 110;
    }

    public static final class Elbow {
      public static final double KP = 0.08;
      public static final double KI = 0.0;
      public static final double KD = 0.0;

      public static final double KV = 0.029;
      public static final double KG = 0.35;
      public static final double KS = 0;
      public static final double KA = 0.0;

      public static final double TOLERANCE_POSITION = 12;
      public static final double TOLERANCE_VELOCITY = 2;
      public static final double MAX_VELOCITY = 130;
      public static final double MAX_ACCELERATION = 180;
    }
  }

  public static final class Controller {
    public static final double MULTIPLIER_SHOULDER = 0.5;
    public static final double MULTIPLIER_ELBOW = 0.2;
  }
}

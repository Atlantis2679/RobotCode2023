package frc.robot;

public final class Constants {
  public static final boolean REPLAY = false;

  public static final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double JOYSTICKS_DEADBAND_VALUE = 0.05;
  }

  public static final class Camera {
    public static final int WIDTH = 352;
    public static final int HEIGHT = 288;
    public static final int FPS = 15;
  }

  public static final class Autos {
    public static final class FreeArm {
      public static final double SHOULDER_RAISE_SPEED = 0.4;
      public static final double TIMEOUT_SECONDS = 0.3;
      public static final double SHOULDER_FREE_ANGLE = -70;
    }

    public static final class ReleaseCube {
      public static final double RELEASE_SPEED = 0.4;
      public static final double RELEASE_TIME_SECONDS = 0.5;
    }

    public static final class releaseCubeSecond {
      public static final double TIMEOUT_SECONDS_RAISE = 6;
      public static final double TIMEOUT_SECONDS_LOWER = 6;

      public static final double DRIVE_TO_DISTANCE = -0.3;
    }

    public static final class releaseCubeThird {
      public static final double TIMEOUT_SECONDS_RAISE = 6;
      public static final double TIMEOUT_SECONDS_LOWER = 6;
    }

    public static final class ReleaseCone {
      public static final double RELEASE_SPEED = 0.4;
      public static final double RELEASE_TIME_SECONDS = 1;
    }

    public static final class releaseConeThird {
      public static final double TIMEOUT_SECONDS_RAISE = 6;
      public static final double TIMEOUT_SECONDS_LOWER = 6;

      public static final double DRIVE_TO_DISTANCE_START = -0.3;
      public static final double DRIVE_TO_DISTANCE_BEFORE_RELEASE = 0.3;
      public static final double DRIVE_TO_DISTANCE_END = -0.4;

    }

    public static final class releaseConeSecond {
      public static final double TIMEOUT_SECONDS_RAISE = 6;
      public static final double TIMEOUT_SECONDS_LOWER = 6;
    }


    public static final class DriveBackwardsOutsideCommunity {
      public static final double DISTANCE_METERS = 3.8;
      public static final double TURN_ENGLE = 180;
      public static final double TIMEOUT_SECONDS = 5;
    }

    public static final class GetOnChargeStationAuto {
      public static final double DRIVE_SPEED = 0.8;
      public static final double FINISH_ANGLE = 17;
      public static final double TIMEOUT_SECONDS = 4.5;
    }

    public static final class BalanceOnChargeStationAuto {
      public static final boolean IS_REVERSED = true;
      public static final double SPEED_TO_CLOSER_CENTER = 0.5;
      public static final double DISTANCE_TO_CLOSER_CENTER = 0.3;
      public static final double GET_CLOSER_TO_CENTER_TIMEOUT = 2;
      
      public static final double MAX_SPEED = 0.27;

      public static final double KP = 0.019;
      public static final double KI = 0;
      public static final double KD = 0.0025;

      public static final double POSITION_TOLERANCE = 1.5;
      public static final double VELOCITY_TOLERANCE = 0.2;
    }
  }
}

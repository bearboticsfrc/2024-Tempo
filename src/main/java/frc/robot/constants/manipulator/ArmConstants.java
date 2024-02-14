package frc.robot.constants.manipulator;

public class ArmConstants {
  public class Motor {
    public static final String MODULE_NAME = "Arm Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorLowerPid {
      public static final double P = 0;
      public static final double MIN_OUTPUT = -0.1;
      public static final double MAX_OUTPUT = 0.1;
      public static final double POSITION_CONVERSION_FACTOR = 360;
      public static final double D = 0;
    }

    public static class MotorRaisePid {
      public static final double P = 0;
      public static final double MIN_OUTPUT = -0.1;
      public static final double MAX_OUTPUT = 0.1;
    }

    public class FeedForward {
      public static final double STATIC = 0;
      public static final double GRAVITY = 0.025;
      public static final double VELOCITY = 0;
    }
  }

  public class MotorFollower {
    public static final String MODULE_NAME = "Arm Motor Follower";
    public static final int MOTOR_PORT = 4;
    public static final int CURRENT_LIMT = 40;
    public static final boolean FOLLOW_INVERTED = true;
  }
}

package frc.robot.constants.manipulator;

public class ClimberConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 2;

  public static class Test {
    public static final double SPEED = 0.5;
    public static final int POSITION_TOLERANCE = 1;
    public static final double WAIT = 2.5;
    public static final double TIMEOUT = WAIT + 1;
  }

  public class Motor {
    public static final String NAME = "Climber Motor";
    public static final int MOTOR_PORT = 14;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;
    public static final float FORWARD_SOFT_LIMIT = 105;
  }

  public class MotorFollower {
    public static final String NAME = "Climber Follower Motor";
    public static final int MOTOR_PORT = 15;
    public static final int CURRENT_LIMT = 40;
    public static final boolean FOLLOW_INVERTED = true;
  }
}

package frc.robot.constants.manipulator;

public class ClimberConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 2;

  public class Motor {
    public static final String NAME = "Climber Motor";
    public static final int MOTOR_PORT = 14;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;
    public static final float FORWARD_SOFT_LIMIT = 110;
  }

  public class MotorFollower {
    public static final String NAME = "Climber Follower Motor";
    public static final int MOTOR_PORT = 15;
    public static final int CURRENT_LIMT = 40;
    public static final boolean FOLLOW_INVERTED = true;
  }
}

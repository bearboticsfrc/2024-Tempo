package frc.robot.constants.manipulator;

public class ClimberConstants {
  public class Motor {
    public static final String MODULE_NAME = "Climber Motor";
    public static final int MOTOR_PORT = 14;
    public static final int CURRENT_LIMT = 20;
    public static final boolean INVERTED = true;

    public class MotorPid {
      public static final double P = 0;
      public static final double Ff = 0.00015;
      public static final double MIN_OUTPUT = -0.2;
      public static final double MAX_OUTPUT = 0.2;
    }
  }

  public class MotorFollower {
    public static final String MODULE_NAME = "Climber Follower Motor";
    public static final int MOTOR_PORT = 15;
    public static final int CURRENT_LIMT = 20;
    public static final boolean FOLLOW_INVERTED = true;
  }
}

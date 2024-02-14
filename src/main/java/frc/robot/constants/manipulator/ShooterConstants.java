package frc.robot.constants.manipulator;

public class ShooterConstants {
  public static final double VELOCITY_TOLERANCE = 25;

  public static class Motor {
    public static final String NAME = "Shooter Motor";
    public static final int MOTOR_PORT = 11;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0; // 0.0001;
      public static final double I = 0; // 0.0000002;
      public static final double I_ZONE = 0; // 100;
      public static final double Ff = 0.00015;
    }
  }

  public static class MotorFollower {
    public static final String NAME = "Shooter Motor Follower";
    public static final int MOTOR_PORT = 7;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;
    public static final boolean FOLLOW_INVERTED = true;
  }
}

package frc.robot.constants.manipulator;

public class ArmConstants {
  public class Motor {
    public static final String MODULE_NAME = "Arm Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMT = 20;
    public static final boolean INVERTED = true;

    public static class MotorLowerPid {
      public static final double P = 0;
      public static final double Ff = 0.1;
      public static final double MIN_OUTPUT = -0.3;
      public static final double MAX_OUTPUT = 0.3;
    }

    public static class MotorRaisePid {
      public static final double P = 0;
      public static final double Ff = 0.1;
      public static final double MIN_OUTPUT = -0.3;
      public static final double MAX_OUTPUT = 0.3;
    }
  }

  public class MotorFollower {
    public static final String MODULE_NAME = "Arm Motor Follower";
    public static final int MOTOR_PORT = 4;
    public static final int CURRENT_LIMT = 20;
    public static final boolean INVERTED = true;
  }
}

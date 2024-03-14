package frc.robot.constants.manipulator;

public class ShooterConstants {
  public static final double MAX_DISTANCE = 5.26;
  public static final double VELOCITY_TOLERANCE = 50;
  public static final double VELOCITY_COMPENSATION = 100;

  public static class UpperMotor {
    public static final String NAME = "Upper Shooter Motor";
    public static final int MOTOR_PORT = 11;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0.0001;
      public static final double Ff = 0.000148;
    }
  }

  public static class LowerMotor {
    public static final String NAME = "Lower Shooter Motor";
    public static final int MOTOR_PORT = 7;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.0001;
      public static final double Ff = 0.00017;
    }
  }
}

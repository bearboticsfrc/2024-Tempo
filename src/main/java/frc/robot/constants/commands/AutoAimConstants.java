package frc.robot.constants.commands;

public class AutoAimConstants {
  public static class RotationPid {
    public static final double P = 0.01;
    public static final double I = 0.01;
    public static final double I_ZONE = 5;
    public static final double D = 0.0005;

    public static final double TOLERANCE = 2;

    public static class ContinuousInput {
      public static final int MIN = -180;
      public static final int MAX = 180;
    }
  }
}

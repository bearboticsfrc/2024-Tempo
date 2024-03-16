package frc.robot.constants.commands;

public class AutoNotePickupConstants {
  public static class YawPid {
    public static final double P = 0.01;
    public static final double I = 0.01;
    public static final double D = 0;

    public static final double TOLERANCE = 1;
  }

  public static class PitchPid {
    public static final double P = 0.05;
    public static final double I = 0;
    public static final double D = 0;

    public static final double TOLERANCE = 2;
  }
}

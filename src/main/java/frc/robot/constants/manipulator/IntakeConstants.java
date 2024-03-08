package frc.robot.constants.manipulator;

import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;

public class IntakeConstants {
  public static final int TOP_BEAM_BREAK_CHANNEL = 6;
  public static final int LEFT_BEAM_BREAK_CHANNEL = 5;
  public static final int RIGHT_BEAM_BREAK_CHANNEL = 7;
  public static final int BOTTOM_BEAM_BREAK_CHANNEL = 8;
  public static final int ROLLER_BEAM_BREAK_CHANNEL = 9;

  public static class Test {
    public static final IntakeSpeed SPEED = IntakeSpeed.HALF;
    public static final double SPEED_TOLERANCE = 0.05;
    public static final double WAIT = 2.5;
    public static final double TIMEOUT = WAIT + 1;
  }

  public static class RollerMotor {
    public static final String NAME = "Roller Motor";
    public static final int MOTOR_PORT = 12;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;
  }

  public static class FeederMotor {
    public static final String NAME = "Feeder Motor";
    public static final int MOTOR_PORT = 6;
    public static final int CURRENT_LIMT = 20;
    public static final boolean INVERTED = true;
  }
}

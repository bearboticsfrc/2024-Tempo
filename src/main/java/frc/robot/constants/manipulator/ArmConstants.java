package frc.robot.constants.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

public class ArmConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 3;
  public static final double POSITION_TOLERANCE = 1;

  public static final double MAX_DISTANCE = 7.0;

  public static final SortedMap<Double, Double> SHOOT_ANGLE_MAP =
      new TreeMap<>(
          Map.ofEntries(
              Map.entry(0.0, 0.0),
              Map.entry(1.3, 0.0),
              Map.entry(1.5, 6.0),
              Map.entry(1.79, 13.5),
              Map.entry(2.00, 17.4),
              Map.entry(2.25, 20.9),
              Map.entry(2.50, 23.5),
              Map.entry(3.00, 27.5),
              Map.entry(3.50, 31.0),
              Map.entry(4.00, 32.5),
              Map.entry(4.50, 35.0),
              Map.entry(5.00, 37.3),
              Map.entry(5.25, 37.4),
              Map.entry(5.5, 38.2),
              Map.entry(5.75, 38.5),
              Map.entry(6.0, 38.7),
              Map.entry(6.25, 38.8),
              Map.entry(6.5, 38.9),
              Map.entry(6.75, 39.0),
              Map.entry(7.0, 39.1),
              Map.entry(7.01, 39.11),
              Map.entry(7.02, 39.12)));

  public class Motor {
    public static final String NAME = "Arm Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMIT = 40;
    public static final double NOMINAL_VOLTAGE = 11.5;
    public static final boolean INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final float FORWARD_SOFT_LIMIT = 80;

    public static final double ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR = 360;
    public static final double RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 360 / 143.077;

    public static final double ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR =
        ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR / 60;
    public static final double RELATIVE_ENCODER_VELOCITY_CONVERSION_FACTOR =
        RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR / 60;

    public static class MotorPid {
      public static final double P = 0.0315;
      public static final double D = 0.0001;
      public static final double MIN_OUTPUT = -0.15;
      public static final double MAX_OUTPUT = 1;
      public static final boolean POSITION_WRAPPING_ENABLED = true;
      public static final double POSITION_WRAPPING_MIN = 0;
      public static final double POSITION_WRAPPING_MAX = 360;
    }

    public static class FeedForward {
      public static final double STATIC = 1.5;
      public static final double GRAVITY = .62;
      public static final double VELOCITY = 0.0;
    }

    public static class TrapezoidProfile {
      public static final Constraints constraints = new Constraints(720 * 8, 180 * 3);
    }
  }
}

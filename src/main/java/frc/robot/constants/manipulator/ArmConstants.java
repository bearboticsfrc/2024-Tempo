package frc.robot.constants.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Map;
import java.util.TreeMap;

public class ArmConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 3;
  public static final double POSITION_TOLERANCE = 1;

  public static final double MAX_DISTANCE = 5.26;

  public static final Map<Double, Double> SHOOT_ANGLE_MAP =
      new TreeMap<>(
          Map.ofEntries(
              Map.entry(0.0, 0.0),
              Map.entry(1.787, 13.5),
              Map.entry(1.95, 17.2),
              Map.entry(2.13, 19.8),
              Map.entry(2.29, 21.27),
              Map.entry(2.46, 21.5),
              Map.entry(2.55, 24.1),
              Map.entry(2.63, 23.8),
              Map.entry(2.81, 27.13),
              Map.entry(2.97, 27.6),
              Map.entry(3.14, 28.9),
              Map.entry(3.2, 30.46),
              Map.entry(3.26, 30.5),
              Map.entry(3.45, 31.1),
              Map.entry(3.63, 31.33),
              Map.entry(3.77, 33.5),
              Map.entry(3.88, 33.8),
              Map.entry(4.0, 34.2),
              Map.entry(4.16, 34.8),
              Map.entry(4.3, 35.6),
              Map.entry(4.46, 36.2),
              Map.entry(4.71, 37.5),
              Map.entry(4.96, 37.9),
              Map.entry(5.25, 38.1),
              Map.entry(5.26, 38.22),
              Map.entry(5.27, 38.24)));

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

package frc.robot.constants.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 3;
  public static final double POSITION_TOLERANCE = 0.5;

  public class Motor {
    public static final String NAME = "Arm Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double POSITION_CONVERSION_FACTOR = 360;
    public static final float FORWARD_SOFT_LIMIT = 80;

    public static class MotorPid {
      public static final double P = 0.03;
      public static final double D = 0; // 0.0001;
      public static final double MIN_OUTPUT = -0.25;
      public static final boolean POSITION_WRAPPING_ENABLED = true;
      public static final double POSITION_WRAPPING_MIN = 0;
      public static final double POSITION_WRAPPING_MAX = 360;
    }

    public static class FeedForward {
      public static final double STATIC = 0.455;
      public static final double GRAVITY = 0.595;
      public static final double VELOCITY = 0;
    }

    public static class TrapezoidProfile {
      public static final double MAX_VELOCITY = 180;
      public static final double MAX_ACCELERATION = 90;
      public static final Constraints CONSTRAINTS = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    }
  }
}

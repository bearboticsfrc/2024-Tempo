package frc.robot.constants.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 3;
  public static final double POSITION_TOLERANCE = 1;
  public static final double FLYWHEEL_RADIUS = 0.0508;
  public static final double ARM_RADIUS = 0.3782314;

  public static final double ROBOT_TO_SHOOTER_PIVOT = .2;
  public static final double SPEAKER_X_OFFSET = .23;
  public static final double SPEAKER_Z_HEIGHT = 2.032;
  public static final double SHOOTER_ANGLE = 56.0;
  public static final double SHOOTER_BASE_HEIGHT = .42;
  public static final double ARM_LENGTH = .425;
  public static final double SHOOTER_TO_SPEAKER_HEIGHT = SPEAKER_Z_HEIGHT - SHOOTER_BASE_HEIGHT;

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

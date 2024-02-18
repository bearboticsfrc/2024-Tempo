package frc.robot.constants.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
  public static final int LIMIT_SWITCH_CHANNEL = 3;
  public static final double POSITION_TOLERANCE = 1;

  public class Motor {
    public static final String NAME = "Arm Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double POSITION_CONVERSION_FACTOR = 360;
    public static final float FORWARD_SOFT_LIMIT = 80;

    public static class MotorPid {
      public static final double P = 0.031;
      public static final double MIN_OUTPUT = -0.4;
    }

    public static class FeedForward {
      public static final double STATIC = 0.75;
      public static final double GRAVITY = 0.6;
      public static final double VELOCITY = 0;
    }

    public static class TrapezoidProfile {
      public static final Constraints constraints = new Constraints(180, 90);
    }
  }
}

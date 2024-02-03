package frc.robot.constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.RateLimiter;

public class DriveConstants {
  public static final ShuffleboardTab DRIVE_SYSTEM_TAB = Shuffleboard.getTab("Drive System");
  public static final ShuffleboardTab MANIPULATOR_SYSTEM_TAB =
      Shuffleboard.getTab("Manipulator System");
  public static final ShuffleboardTab COMPETITION_TAB = Shuffleboard.getTab("Competition");
  public static final ShuffleboardTab TEST_TAB = Shuffleboard.getTab("Test");

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  /*
   * Max free spin for the Spark (taken from docs)
   */
  public static final int MAX_MOTOR_FREE_SPEED_RPM = 6784;

  public static final double WHEEL_CIRCUMFERENCE = RobotConstants.WHEEL_DIAMETER * Math.PI;

  public static final double DRIVE_GEAR_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
      (MAX_MOTOR_FREE_SPEED_RPM / 60 * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_REDUCTION;

  /** Max drive velocity in meters/sec */
  public static final double MAX_VELOCITY =
      (MAX_MOTOR_FREE_SPEED_RPM / 60.0 * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_REDUCTION;

  public static final double DRIVE_VELOCITY = MAX_VELOCITY / 2;

  public static final double MAX_ACCELERATION_PER_SECOND = 4;
  public static final double MAX_DECELERATION_PER_SECOND = 4;
  public static final double MAX_ANGULAR_ACCELERATION_PER_SECOND = 6 * (Math.PI * 2);
  public static final double MAX_ANGULAR_DECELERATION_PER_SECOND = 20;

  public static final RateLimiter X_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter Y_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_DECELERATION_PER_SECOND);

  public static final RateLimiter TURNING_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ANGULAR_ACCELERATION_PER_SECOND, MAX_ANGULAR_DECELERATION_PER_SECOND);

  public enum SpeedMode {
    TURBO(2),
    NORMAL(1),
    TURTLE(0.5);

    private final double maxSpeedMultiplier;

    private SpeedMode(double maxSpeedMultiplier) {
      this.maxSpeedMultiplier = maxSpeedMultiplier;
    }

    public double getMaxSpeedMultiplier() {
      return maxSpeedMultiplier;
    }

    public double getMaxSpeed() {
      return DRIVE_VELOCITY * getMaxSpeedMultiplier();
    }
  }
}

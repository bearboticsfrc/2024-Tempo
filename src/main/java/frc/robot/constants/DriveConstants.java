package frc.robot.constants;

import frc.robot.util.RateLimiter;

/** Constants related to the drivetrain and control. */
public class DriveConstants {
  // Controller ports
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // Max free spin for the Spark (taken from docs)
  public static final int MAX_MOTOR_FREE_SPEED_RPM = 5676;

  // Wheel circumference based on the wheel diameter constant
  public static final double WHEEL_CIRCUMFERENCE = RobotConstants.WHEEL_DIAMETER * Math.PI;

  // Drive and steer gear reduction ratios
  public static final double DRIVE_GEAR_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
  public static final double STEER_DRIVE_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  // Drive wheel free speed in meters per second
  public static final double DRIVE_WHEEL_FREE_SPEED_MPS =
      (MAX_MOTOR_FREE_SPEED_RPM / 60 * WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_REDUCTION;

  // Max drive velocity in meters per second
  public static final double MAX_VELOCITY =
      (MAX_MOTOR_FREE_SPEED_RPM / 60.0 * WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_REDUCTION;

  // Default drive velocity
  public static final double DRIVE_VELOCITY = MAX_VELOCITY / 2;

  // Max linear and angular acceleration and deceleration limits
  public static final double MAX_ACCELERATION_PER_SECOND = 7;
  public static final double MAX_ANGULAR_ACCELERATION_PER_SECOND = 1.5 * (Math.PI * 2);
  public static final double MAX_ANGULAR_DECELERATION_PER_SECOND = 20;

  // Rate limiters for acceleration
  public static final RateLimiter X_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_ACCELERATION_PER_SECOND);

  public static final RateLimiter Y_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ACCELERATION_PER_SECOND, MAX_ACCELERATION_PER_SECOND);

  public static final RateLimiter TURNING_ACCELERATION_LIMITER =
      new RateLimiter(MAX_ANGULAR_ACCELERATION_PER_SECOND, MAX_ANGULAR_DECELERATION_PER_SECOND);

  // Enum for different speed modes
  public enum SpeedMode {
    TURBO(2),
    NORMAL(1),
    TURTLE(0.5);

    private final double maxSpeedMultiplier;

    private SpeedMode(double maxSpeedMultiplier) {
      this.maxSpeedMultiplier = maxSpeedMultiplier;
    }

    /**
     * Gets the maximum speed multiplier for the speed mode.
     *
     * @return The maximum speed multiplier.
     */
    public double getMaxSpeedMultiplier() {
      return maxSpeedMultiplier;
    }

    /**
     * Gets the maximum speed for the speed mode.
     *
     * @return The maximum speed.
     */
    public double getMaxSpeed() {
      return DRIVE_VELOCITY * getMaxSpeedMultiplier();
    }
  }
}

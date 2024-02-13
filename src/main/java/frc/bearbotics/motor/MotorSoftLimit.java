package frc.bearbotics.motor;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.math.geometry.Rotation2d;

/** Builder class for configuring motor soft limit parameters. */
public class MotorSoftLimit {
  private SoftLimitDirection direction;
  private Rotation2d limit;

  /**
   * Get the soft limit value.
   *
   * @return The soft limit value.
   */
  public Rotation2d getLimit() {
    return limit;
  }

  /**
   * Set the soft limit value.
   *
   * @param limit The desired soft limit value.
   * @return This MotorSoftLimit instance for method chaining.
   */
  public MotorSoftLimit withLimit(Rotation2d limit) {
    this.limit = limit;
    return this;
  }

  /**
   * Get the soft limit direction.
   *
   * @return The soft limit direction.
   */
  public SoftLimitDirection getDirection() {
    return direction;
  }

  /**
   * Set the soft limit direction.
   *
   * @param direction The desired soft limit direction.
   * @return This MotorSoftLimit instance for method chaining.
   */
  public MotorSoftLimit withDirection(SoftLimitDirection direction) {
    this.direction = direction;
    return this;
  }
}

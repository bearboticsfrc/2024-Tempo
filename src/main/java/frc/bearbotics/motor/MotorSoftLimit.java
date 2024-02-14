package frc.bearbotics.motor;

import com.revrobotics.CANSparkBase.SoftLimitDirection;

/** Builder class for configuring motor soft limit parameters. */
public class MotorSoftLimit {
  private SoftLimitDirection direction;
  private float limit;

  /**
   * Get the soft limit value.
   *
   * @return The soft limit value.
   */
  public float getLimit() {
    return limit;
  }

  /**
   * Set the soft limit value.
   *
   * @param limit The desired soft limit value.
   * @return This MotorSoftLimit instance for method chaining.
   */
  public MotorSoftLimit withLimit(float limit) {
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

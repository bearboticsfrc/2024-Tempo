package frc.bearbotics.motor;

/** Builder class for configuring motor PID parameters. */
public class MotorPidBuilder {
  private double p;
  private double i;
  private double iZone;
  private double d;
  private double ff;
  private double minOutput = -1;
  private double maxOutput = 1;
  private boolean positionPidWrappingEnabled;
  private double positionPidWrappingMin = -1;
  private double positionPidWrappingMax = 1;

  /**
   * Get the proportional gain (P) value.
   *
   * @return The P value.
   */
  public double getP() {
    return p;
  }

  /**
   * Set the proportional gain (P) value.
   *
   * @param p The desired P value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withP(double p) {
    this.p = p;
    return this;
  }

  /**
   * Get the integral gain (I) value.
   *
   * @return The I value.
   */
  public double getI() {
    return i;
  }

  /**
   * Set the integral gain (I) value.
   *
   * @param i The desired I value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withI(double i) {
    this.i = i;
    return this;
  }

  /**
   * Get the integral zone (I zone) value.
   *
   * @return The I zone value.
   */
  public double getIZone() {
    return iZone;
  }

  /**
   * Set the integral zone gain (I zone) value.
   *
   * @param iZone The desired I zone value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withIZone(double iZone) {
    this.iZone = iZone;
    return this;
  }

  /**
   * Get the derivative gain (D) value.
   *
   * @return The D value.
   */
  public double getD() {
    return d;
  }

  /**
   * Set the derivative gain (D) value.
   *
   * @param d The desired D value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withD(double d) {
    this.d = d;
    return this;
  }

  /**
   * Get the feedforward (FF) value.
   *
   * @return The FF value.
   */
  public double getFf() {
    return ff;
  }

  /**
   * Set the feedforward (FF) value.
   *
   * @param ff The desired FF value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withFf(double ff) {
    this.ff = ff;
    return this;
  }

  /**
   * Get the minimum output value for the PID controller.
   *
   * @return The minimum output value.
   */
  public double getMinOutput() {
    return minOutput;
  }

  /**
   * Set the minimum output value for the PID controller.
   *
   * @param minOutput The desired minimum output value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withMinOutput(double minOutput) {
    this.minOutput = minOutput;
    return this;
  }

  /**
   * Get the maximum output value for the PID controller.
   *
   * @return The maximum output value.
   */
  public double getMaxOutput() {
    return maxOutput;
  }

  /**
   * Set the maximum output value for the PID controller.
   *
   * @param maxOutput The desired maximum output value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
    return this;
  }

  /**
   * Check if position PID wrapping is enabled.
   *
   * @return True if position PID wrapping is enabled, false otherwise.
   */
  public boolean isPositionPidWrappingEnabled() {
    return positionPidWrappingEnabled;
  }

  /**
   * Set whether position PID wrapping is enabled.
   *
   * @param enabled True to enable position PID wrapping, false otherwise.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withPositionPidWrappingEnabled(boolean enabled) {
    this.positionPidWrappingEnabled = enabled;
    return this;
  }

  /**
   * Get the minimum input value for position PID wrapping.
   *
   * @return The minimum input value for position PID wrapping.
   */
  public double getPositionPidWrappingMin() {
    return positionPidWrappingMin;
  }

  /**
   * Set the minimum input value for position PID wrapping.
   *
   * @param positionPidWrappingMin The desired minimum input value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withPositionPidWrappingMin(double positionPidWrappingMin) {
    this.positionPidWrappingMin = positionPidWrappingMin;
    return this;
  }

  /**
   * Get the maximum input value for position PID wrapping.
   *
   * @return The maximum input value for position PID wrapping.
   */
  public double getPositionPidWrappingMax() {
    return positionPidWrappingMax;
  }

  /**
   * Set the maximum input value for position PID wrapping.
   *
   * @param positionPidWrappingMax The desired maximum input value.
   * @return This MotorPidBuilder instance for method chaining.
   */
  public MotorPidBuilder withPositionPidWrappingMax(double positionPidWrappingMax) {
    this.positionPidWrappingMax = positionPidWrappingMax;
    return this;
  }
}

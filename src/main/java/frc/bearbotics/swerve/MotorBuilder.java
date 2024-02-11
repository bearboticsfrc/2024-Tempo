package frc.bearbotics.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import frc.bearbotics.swerve.cancoder.CANCoders.CANCoderBuilder;

/** Builder class for configuring motor parameters. */
public class MotorBuilder {
  private String name;
  private String moduleName;
  private int motorPort;
  private boolean motorInverted;
  private boolean encoderInverted;
  private CANCoderBuilder absoluteEncoder;
  private MotorPidBuilder motorPID;
  private double nominalVoltage = 12;
  private int currentLimit;
  private double positionConversionFactor = 1;
  private double velocityConversionFactor = 1;
  private IdleMode idleMode = IdleMode.kBrake;
  private boolean followInverted;
  private MotorPidBuilder[] pidSlots = new MotorPidBuilder[2];

  /**
   * Get the configured name of the motor.
   *
   * @return The motor name.
   */
  public String getName() {
    return name;
  }

  /**
   * Set the name of the motor.
   *
   * @param name The desired motor name.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withName(String name) {
    this.name = name;
    return this;
  }

  /**
   * Get the configured module name of the motor.
   *
   * @return The motor module name.
   */
  public String getModuleName() {
    return moduleName;
  }

  /**
   * Set the module name of the motor.
   *
   * @param moduleName The desired motor module name.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withModuleName(String moduleName) {
    this.moduleName = moduleName;
    return this;
  }

  /**
   * Get the configured motor port number.
   *
   * @return The motor port number.
   */
  public int getMotorPort() {
    return motorPort;
  }

  /**
   * Set the motor port number.
   *
   * @param motorPort The desired motor port number.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withMotorPort(int motorPort) {
    this.motorPort = motorPort;
    return this;
  }

  /**
   * Check if the motor is configured to be inverted.
   *
   * @return True if the motor is inverted, false otherwise.
   */
  public boolean isMotorInverted() {
    return motorInverted;
  }

  /**
   * Set whether the motor should be inverted.
   *
   * @param motorInverted True to invert the motor, false otherwise.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withMotorInverted(boolean motorInverted) {
    this.motorInverted = motorInverted;
    return this;
  }

  /**
   * Check if the motor encoder is configured to be inverted.
   *
   * @return True if the encoder is inverted, false otherwise.
   */
  public boolean isEncoderInverted() {
    return encoderInverted;
  }

  /**
   * Set whether the motor encoder should be inverted.
   *
   * @param encoderInverted True to invert the encoder, false otherwise.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withEncoderInverted(boolean encoderInverted) {
    this.encoderInverted = encoderInverted;
    return this;
  }

  /**
   * Get the current idle mode setting.
   *
   * @return The current idle mode.
   */
  public IdleMode getIdleMode() {
    return idleMode;
  }

  /**
   * Set the idle mode for the motor.
   *
   * @param idleMode The desired idle mode.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withIdleMode(IdleMode idleMode) {
    this.idleMode = idleMode;
    return this;
  }

  /**
   * Get the configured absolute encoder for the motor.
   *
   * @return The configured CANCoder for absolute encoding.
   */
  public CANCoderBuilder getAbsoluteEncoder() {
    return absoluteEncoder;
  }

  /**
   * Set the absolute encoder for the motor.
   *
   * @param canCoder The CANCoderBuilder for configuring the absolute encoder.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withAbsoluteEncoder(CANCoderBuilder canCoder) {
    this.absoluteEncoder = canCoder;
    return this;
  }

  /**
   * Get the configured MotorPidBuilder for the motor.
   *
   * @return The configured MotorPidBuilder.
   */
  public MotorPidBuilder getMotorPID() {
    return motorPID;
  }

  /**
   * Set the MotorPidBuilder for the motor.
   *
   * @param motorPID The desired MotorPidBuilder.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withMotorPID(MotorPidBuilder motorPID) {
    this.motorPID = motorPID;
    return this;
  }

  /**
   * Set the motor PID configuration for a specific slot.
   *
   * @param motorPid The MotorPidBuilder for configuring PID parameters.
   * @param slot The PID slot to configure.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withMotorPid(MotorPidBuilder motorPid, int slot) {
    pidSlots[slot] = motorPid;
    return this;
  }

  /**
   * Get the motor PID configuration for a specific slot.
   *
   * @param slot The PID slot to retrieve.
   * @return The MotorPidBuilder for the specified slot.
   */
  public MotorPidBuilder getMotorPid(int slot) {
    return pidSlots[slot];
  }

  /**
   * Get the configured nominal voltage for the motor.
   *
   * @return The nominal voltage.
   */
  public double getNominalVoltage() {
    return nominalVoltage;
  }

  /**
   * Set the nominal voltage for the motor.
   *
   * @param nominalVoltage The desired nominal voltage.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withNominalVoltage(double nominalVoltage) {
    this.nominalVoltage = nominalVoltage;
    return this;
  }

  /**
   * Get the configured current limit for the motor.
   *
   * @return The current limit.
   */
  public int getCurrentLimit() {
    return currentLimit;
  }

  /**
   * Set the current limit for the motor.
   *
   * @param currentLimit The desired current limit.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withCurrentLimit(int currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  /**
   * Get the configured position conversion factor.
   *
   * @return The position conversion factor.
   */
  public double getPositionConversionFactor() {
    return positionConversionFactor;
  }

  /**
   * Set the position conversion factor for the motor.
   *
   * @param positionConversionFactor The desired position conversion factor.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withPositionConversionFactor(double positionConversionFactor) {
    this.positionConversionFactor = positionConversionFactor;
    return this;
  }

  /**
   * Check if the motor follower is configured to be inverted.
   *
   * @return True if the motor follower is inverted, false otherwise.
   */
  public boolean isFollowInverted() {
    return followInverted;
  }

  /**
   * Set whether the motor follower should be inverted.
   *
   * @param followInverted True to invert the motor follower, false otherwise.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withFollowInverted(boolean followInverted) {
    this.followInverted = followInverted;
    return this;
  }

  /**
   * Get the configured velocity conversion factor.
   *
   * @return The velocity conversion factor.
   */
  public double getVelocityConversionFactor() {
    return velocityConversionFactor;
  }

  /**
   * Set the velocity conversion factor for the motor.
   *
   * @param velocityConversionFactor The desired velocity conversion factor.
   * @return This MotorBuilder instance for method chaining.
   */
  public MotorBuilder withVelocityConversionFactor(double velocityConversionFactor) {
    this.velocityConversionFactor = velocityConversionFactor;
    return this;
  }
}

package frc.bearbotics.swerve;

import frc.bearbotics.motor.MotorBuilder;

/**
 * Builder class for constructing instances of the {@link SwerveModule} class. Allows configuring
 * the parameters of a swerve module before creating an instance.
 */
public class SwerveModuleBuilder {
  private String moduleName;

  private MotorBuilder driveMotor;
  private MotorBuilder pivotMotor;

  /**
   * Gets the name of the swerve module.
   *
   * @return The name of the swerve module.
   */
  public String getModuleName() {
    return moduleName;
  }

  /**
   * Sets the name of the swerve module.
   *
   * @param moduleName The name to set.
   * @return The current instance of the builder for method chaining.
   */
  public SwerveModuleBuilder setModuleName(String moduleName) {
    this.moduleName = moduleName;
    return this;
  }

  /**
   * Gets the builder for the drive motor.
   *
   * @return The builder for the drive motor.
   */
  public MotorBuilder getDriveMotor() {
    return driveMotor;
  }

  /**
   * Sets the builder for the drive motor.
   *
   * @param driveMotor The builder for the drive motor.
   * @return The current instance of the builder for method chaining.
   */
  public SwerveModuleBuilder setDriveMotor(MotorBuilder driveMotor) {
    this.driveMotor = driveMotor;
    return this;
  }

  /**
   * Gets the builder for the pivot motor.
   *
   * @return The builder for the pivot motor.
   */
  public MotorBuilder getPivotMotor() {
    return pivotMotor;
  }

  /**
   * Sets the builder for the pivot motor.
   *
   * @param pivotMotor The builder for the pivot motor.
   * @return The current instance of the builder for method chaining.
   */
  public SwerveModuleBuilder setPivotMotor(MotorBuilder pivotMotor) {
    this.pivotMotor = pivotMotor;
    return this;
  }
}

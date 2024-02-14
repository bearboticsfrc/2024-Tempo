package frc.bearbotics.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.cancoder.CANCoders;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

/**
 * Represents a Swerve Module in a robotics application. A Swerve Module consists of a drive motor
 * and a pivot motor.
 */
public class SwerveModule {

  // Flag to enable/disable Shuffleboard integration
  private final boolean SHUFFLEBOARD_ENABLED = true;

  // Name of the swerve module
  private String moduleName;

  // Drive motor and pivot motor
  private CANSparkFlex driveMotor;
  private CANSparkMax pivotMotor;

  // Encoders for drive and pivot motors
  private RelativeEncoder driveMotorEncoder;
  private RelativeEncoder pivotMotorRelativeEncoder;
  private CANcoder pivotMotorAbsoluteEncoder;

  // PID controllers for drive and pivot motors
  private SparkPIDController driveMotorPIDController;
  private SparkPIDController pivotMotorPIDController;

  // Reference angle for the swerve module
  private Rotation2d referenceAngle = new Rotation2d();

  // Data logs for various motor properties
  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  /**
   * Initializes the swerve module.
   *
   * @param swerveModule The builder that contains all constants.
   * @param shuffleboardTab The shuffleboard tab to use.
   */
  public SwerveModule(SwerveModuleBuilder swerveModule, ShuffleboardTab shuffleboardTab) {
    this.moduleName = swerveModule.getModuleName();

    this.driveMotor =
        new CANSparkFlex(
            swerveModule.getDriveMotor().getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    this.pivotMotor =
        new CANSparkMax(
            swerveModule.getPivotMotor().getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    this.driveMotorEncoder = driveMotor.getEncoder();
    this.pivotMotorRelativeEncoder = pivotMotor.getEncoder();

    MotorConfig.fromMotorConstants(driveMotor, driveMotorEncoder, swerveModule.getDriveMotor())
        .configureMotor()
        .configureEncoder(Rotation2d.fromRadians(0))
        .configurePid()
        .burnFlash();

    MotorConfig.fromMotorConstants(
            pivotMotor, pivotMotorRelativeEncoder, swerveModule.getPivotMotor())
        .configureMotor()
        .configureCanCoder()
        .configurePid()
        .burnFlash();

    this.pivotMotorAbsoluteEncoder =
        CANCoders.getInstance().get(swerveModule.getPivotMotor().getCanCoderBuilder().getId());

    MotorConfig.fromMotorConstants(
            pivotMotor, pivotMotorRelativeEncoder, swerveModule.getPivotMotor())
        .configureEncoder(getAbsoluteAngle())
        .burnFlash();

    this.driveMotorPIDController = driveMotor.getPIDController();
    this.pivotMotorPIDController = pivotMotor.getPIDController();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(shuffleboardTab);
    }

    setupDataLogging(DataLogManager.getLog());
  }

  @Override
  public String toString() {
    return moduleName;
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Vel", moduleName), this::getDriveVelocity)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Drive Out", moduleName), driveMotor::getAppliedOutput)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Pos", moduleName), this::getDistance)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Rel Deg", moduleName), () -> getRelativeAngle().getDegrees())
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Abs Deg", moduleName), () -> getAbsoluteAngle().getDegrees())
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Ref Deg", moduleName), () -> referenceAngle.getDegrees())
        .withSize(1, 1);
  }

  /**
   * Setup data logging.
   *
   * @param log The data log to use.
   */
  private void setupDataLogging(DataLog log) {
    for (String motorType : new String[] {"DRIVE", "PIVOT"}) {
      String pathMotorType = motorType.toLowerCase();

      if (motorType.equals("PIVOT")) {
        dataLogs.put(
            String.format("%s_MOTOR_POSITION", motorType),
            new DoubleLogEntry(
                log, String.format("/drive/%s/%s_motor/position", moduleName, pathMotorType)));
      }

      dataLogs.put(
          String.format("%s_MOTOR_CURRENT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_VELOCITY", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/velocity", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_APPLIED_OUTPUT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/applied_output", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_TEMPERATURE", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/temperature", moduleName, pathMotorType)));
    }
  }

  /** Updates data logs */
  public void updateDataLogs() {
    for (Entry<String, DoubleLogEntry> entry : dataLogs.entrySet()) {
      CANSparkBase motor = entry.getKey().startsWith("PIVOT") ? pivotMotor : driveMotor;
      String property =
          entry
              .getKey()
              .replaceAll(
                  "^(PIVOT_MOTOR_|DRIVE_MOTOR_)", ""); // e.g. PIVOT_MOTOR_POSITION -> POSITION

      DoubleSupplier propertySupplier = getPropertySupplier(motor, property);
      entry.getValue().append(propertySupplier.getAsDouble());
    }
  }

  /**
   * Returns the respective getter for <b>property</b>.
   *
   * @param property The property.
   * @return The getter, wrapped as a DoubleSupplier.
   */
  public DoubleSupplier getPropertySupplier(CANSparkBase motor, String property) {
    switch (property) {
      case "CURRENT":
        return motor::getOutputCurrent;
      case "VELOCITY":
        return motor.getEncoder()::getVelocity;
      case "APPLIED_OUTPUT":
        return motor::getAppliedOutput;
      case "TEMPERATURE":
        return motor::getMotorTemperature;
      case "POSITION":
        return pivotMotorRelativeEncoder::getPosition;
      default:
        throw new IllegalArgumentException("Unknown motor property: " + property);
    }
  }

  /**
   * Returns the current angle from the relative encoder.
   *
   * @return The angle, wrapped as a Rotation2d.
   */
  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromRadians(
        MathUtil.inputModulus(pivotMotorRelativeEncoder.getPosition(), 0, 2 * Math.PI));
  }

  /**
   * Returns the current angle from the absolute encoder
   *
   * @return The angle, wrapped as a Rotation2d.
   */
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(pivotMotorAbsoluteEncoder.getAbsolutePosition().getValue() * 360);
  }

  /**
   * Returns the current drive veloticty.
   *
   * @return The veloticty.
   */
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  /**
   * Returns a Rotation2d representation of the position of the drive encoder.
   *
   * @return The position.
   */
  public double getDistance() {
    return driveMotorEncoder.getPosition();
  }

  /**
   * Returns the position of the swerve module.
   *
   * @return The position, wrapped as a SwerveModulePosition.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotorEncoder.getPosition(), getAbsoluteAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getRelativeAngle());
  }

  /**
   * Sets the relative angle in radians.
   *
   * @param state The state of the swerve module.
   */
  public void set(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getRelativeAngle());

    pivotMotorPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
    driveMotorPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

    referenceAngle = state.angle;
  }
  /**
   * Builder class for constructing instances of the {@link SwerveModule} class. Allows configuring
   * the parameters of a swerve module before creating an instance.
   */
  public static class SwerveModuleBuilder {
    // Name of the swerve module
    private String moduleName;

    // Builder for the drive motor
    private MotorBuilder driveMotor;

    // Builder for the pivot motor
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
}

package frc.bearbotics.motor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.bearbotics.motor.cancoder.CANCoders;
import frc.robot.util.RevUtil;

/** Configuration class for motor parameters, including feedback sensors and builders. */
public class MotorConfig {

  private StringLogEntry logEntry =
      new StringLogEntry(DataLogManager.getLog(), "MotorConfig/Config");

  private CANSparkBase motor;
  private MotorFeedbackSensor motorEncoder;
  private MotorBuilder motorBuilder;

  /**
   * Constructs a MotorConfig with motor, encoder, and MotorBuilder.
   *
   * @param motor The CANSparkBase motor.
   * @param motorEncoder The MotorFeedbackSensor for motor feedback.
   * @param motorBuilder The MotorBuilder for configuring motor parameters.
   */
  public MotorConfig(
      CANSparkBase motor, MotorFeedbackSensor motorEncoder, MotorBuilder motorBuilder) {
    this(motor, motorBuilder);
    this.motorEncoder = motorEncoder;
  }

  /**
   * Constructs a MotorConfig with motor and MotorBuilder.
   *
   * @param motor The CANSparkBase motor.
   * @param motorBuilder The MotorBuilder for configuring motor parameters.
   */
  public MotorConfig(CANSparkBase motor, MotorBuilder motorBuilder) {
    this.motor = motor;
    this.motorBuilder = motorBuilder;
  }

  private String getMotorDescription() {
    return motorBuilder.getName() == null
        ? motorBuilder.getModuleName()
        : String.format("%s - %s", motorBuilder.getModuleName(), motorBuilder.getName());
  }
  /**
   * Configures generic motor parameters, including inversion, idle mode, voltage compensation,
   * current limit, or low and high soft limits.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureMotor() {
    motor.setInverted(motorBuilder.isMotorInverted());

    String motorDescription = getMotorDescription();
    RevUtil.checkRevError(motor.setIdleMode(motorBuilder.getIdleMode()));
    RevUtil.checkRevError(motor.enableVoltageCompensation(motorBuilder.getNominalVoltage()));
    RevUtil.checkRevError(motor.setSmartCurrentLimit(motorBuilder.getCurrentLimit()));
    RevUtil.setPeriodicFramePeriodHigh(motor, motorDescription);

    String message =
        String.format(
            "[MotorConfig.configureMotor] %s:\n\tSet idle mode -> %s.\n\tSet voltage compensation -> %s.\n\tSet smart current limit -> %s.\n\t",
            motorDescription,
            motorBuilder.getIdleMode(),
            motorBuilder.getNominalVoltage(),
            motorBuilder.getCurrentLimit());

    if (motorBuilder.getReverseSoftLimit() != null) {
      MotorSoftLimit reverseSoftLimit = motorBuilder.getReverseSoftLimit();
      RevUtil.checkRevError(
          motor.setSoftLimit(reverseSoftLimit.getDirection(), reverseSoftLimit.getLimit()));

      message +=
          String.format(
              "Reverse soft limit -> true\n\t\tReverse soft limit direction -> %s\n\t\tReverse soft limit -> %s\n\t",
              reverseSoftLimit.getDirection(), reverseSoftLimit.getLimit());
    }

    if (motorBuilder.getForwardSoftLimit() != null) {
      MotorSoftLimit forwardSoftLimit = motorBuilder.getForwardSoftLimit();
      RevUtil.checkRevError(
          motor.setSoftLimit(forwardSoftLimit.getDirection(), forwardSoftLimit.getLimit()));

      message +=
          String.format(
              "Forward soft limit -> true\n\t\tForward soft limit direction -> %s\n\t\tForward soft limit -> %s\n\t",
              forwardSoftLimit.getDirection(), forwardSoftLimit.getLimit());
    }

    logEntry.append(message);

    return this;
  }

  /**
   * Configures the motor encoder parameters, including initial position, feedback device, and
   * conversion factors.
   *
   * @param initialPosition The initial position for the motor encoder.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureEncoder(Rotation2d initialPosition) {
    String message =
        String.format(
            "[MotorConfig.configureEncoder] %s - %s:\n\t",
            getMotorDescription(), motorEncoder.getClass());

    if (motorEncoder instanceof RelativeEncoder) {
      ((RelativeEncoder) motorEncoder).setPosition(initialPosition.getRadians());
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()));
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      RevUtil.checkRevError(
          ((AbsoluteEncoder) motorEncoder).setInverted(motorBuilder.isEncoderInverted()));
      RevUtil.checkRevError(
          ((AbsoluteEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((AbsoluteEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()));
    } else if (motorEncoder instanceof SparkAbsoluteEncoder) {
      ((SparkAbsoluteEncoder) motorEncoder).setInverted(motorBuilder.isEncoderInverted());
      RevUtil.checkRevError(
          ((SparkAbsoluteEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((SparkAbsoluteEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()));
    }

    message +=
        String.format(
            "Inital position -> %s\n\tPosition conversion factor -> %s\n\tVelocity conversion factor -> %s\n\t",
            initialPosition.getDegrees(),
            motorBuilder.getPositionConversionFactor(),
            motorBuilder.getVelocityConversionFactor());

    if (motor.getPIDController() != null) {
      message += String.format("Set feedback device -> %s\n", motorEncoder.getClass());
      RevUtil.checkRevError(motor.getPIDController().setFeedbackDevice(motorEncoder));
    }

    logEntry.append(message);
    return this;
  }

  /**
   * Configures the can coder parameters using CANCoders.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureCanCoder() {
    CANCoders.getInstance().configure(motorBuilder.getCanCoderBuilder());

    String message =
        String.format(
            "[MotorConfig.configureCanCoder] %s:\n\tSet ID -> %s\n\tSet offset angle (degrees) -> %s\n",
            getMotorDescription(),
            motorBuilder.getCanCoderBuilder().getId(),
            motorBuilder.getCanCoderBuilder().getOffsetDegrees().getDegrees());
    logEntry.append(message);

    return this;
  }

  /**
   * Configures the PID parameters for the motor.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configurePid() {
    return configurePid(0);
  }

  /**
   * Configures the PID parameters for the motor in a specific slot.
   *
   * @param slot The PID slot to configure.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configurePid(int slot) {
    MotorPidBuilder motorPid = motorBuilder.getMotorPid(slot);
    SparkPIDController motorPIDController = motor.getPIDController();

    RevUtil.checkRevError(motorPIDController.setP(motorPid.getP(), slot));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getI(), slot));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getIZone(), slot));
    RevUtil.checkRevError(motorPIDController.setD(motorPid.getD(), slot));
    RevUtil.checkRevError(motorPIDController.setFF(motorPid.getFf(), slot));
    RevUtil.checkRevError(
        motorPIDController.setOutputRange(motorPid.getMinOutput(), motorPid.getMaxOutput(), slot));

    if (motorPid.isPositionPidWrappingEnabled()) {
      configurePositionalPidWrapping(motorPid);
    }

    String message =
        String.format(
            "[MotorConfig.configurePid] %s:\n\tSet P -> %s\n\tSet I -> %s\n\tSet I Zone -> %s\n\tSet D -> %s\n\tSet FF -> %s\n\tSet min output -> %s\n\tSet max output -> %s\n",
            getMotorDescription(),
            motorPid.getP(),
            motorPid.getI(),
            motorPid.getIZone(),
            motorPid.getD(),
            motorPid.getFf(),
            motorPid.getMinOutput(),
            motorPid.getMaxOutput());
    logEntry.append(message);

    return this;
  }

  /**
   * Configures positional PID wrapping parameters for the motor.
   *
   * @param motorPid The MotorPidBuilder containing PID parameters.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configurePositionalPidWrapping(MotorPidBuilder motorPid) {
    SparkPIDController motorPIDController = motor.getPIDController();
    boolean positionPidWrappingEnabled = motorPid.isPositionPidWrappingEnabled();

    RevUtil.checkRevError(
        motorPIDController.setPositionPIDWrappingEnabled(positionPidWrappingEnabled));
    RevUtil.checkRevError(
        motorPIDController.setPositionPIDWrappingMinInput(motorPid.getPositionPidWrappingMin()));
    RevUtil.checkRevError(
        motorPIDController.setPositionPIDWrappingMaxInput(motorPid.getPositionPidWrappingMax()));

    String message =
        String.format(
            "[MotorConfig.configurePositionalPidWrapping] %s:\n\tSet positional PID wrapping enabled -> true\n\tSet positional PID wrapping min input -> %s\n\tSet positional PID wrapping max input -> %s\n",
            getMotorDescription(),
            motorPid.getPositionPidWrappingMin(),
            motorPid.getPositionPidWrappingMax());
    logEntry.append(message);

    return this;
  }

  /**
   * Sets the motor to follow another motor.
   *
   * @param leaderMotor The leader motor to follow.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig follow(CANSparkBase leaderMotor) {
    motor.follow(leaderMotor, motorBuilder.isFollowInverted());

    String message =
        String.format(
            "[MotorConfig.follow] %s:\n\tSet follow -> true\n\tSet leader (ID) -> %s\n\tSet follow inverted -> %s\n",
            getMotorDescription(), leaderMotor.getDeviceId(), motorBuilder.isFollowInverted());
    logEntry.append(message);

    return this;
  }

  /** Burns parameters onto the motor flash. */
  public void burnFlash() {
    String message =
        String.format(
            "[MotorConfig.burnFlash] %s:\n\tBurn flash delay (seconds) -> 0.25\n",
            getMotorDescription(), 0.25);
    logEntry.append(message);

    Timer.delay(0.25);
    RevUtil.checkRevError(motor.burnFlash());
    Timer.delay(0.25);
    // Burn parameters onto motor flash
    // might not work, needs a delay after setting values
  }

  /**
   * Creates a MotorConfig instance from motor constants, including motor, encoder, and
   * MotorBuilder.
   *
   * @param motor The CANSparkBase motor.
   * @param motorEncoder The MotorFeedbackSensor for motor feedback.
   * @param motorBuilder The MotorBuilder for configuring motor parameters.
   * @return A new MotorConfig instance.
   */
  public static MotorConfig fromMotorConstants(
      CANSparkBase motor, MotorFeedbackSensor motorEncoder, MotorBuilder motorBuilder) {
    return new MotorConfig(motor, motorEncoder, motorBuilder);
  }

  /**
   * Creates a MotorConfig instance from motor constants, including motor and MotorBuilder.
   *
   * @param motor The CANSparkBase motor.
   * @param motorBuilder The MotorBuilder for configuring motor parameters.
   * @return A new MotorConfig instance.
   */
  public static MotorConfig fromMotorConstants(CANSparkBase motor, MotorBuilder motorBuilder) {
    return new MotorConfig(motor, motorBuilder);
  }
}

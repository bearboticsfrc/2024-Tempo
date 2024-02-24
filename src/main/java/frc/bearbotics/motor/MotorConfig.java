package frc.bearbotics.motor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
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

  /**
   * Configures generic motor parameters, including inversion, idle mode, voltage compensation,
   * current limit, or low and high soft limits.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureMotor() {
    motor.setInverted(motorBuilder.isMotorInverted());

    String motorDescription = motorBuilder.getName();
    RevUtil.checkRevError(
        motor.setIdleMode(motorBuilder.getIdleMode()),
        String.format(
            "[MotorConfig.configureMotor]: Failed to set idle mode to %s",
            motorBuilder.getIdleMode()));
    RevUtil.checkRevError(
        motor.enableVoltageCompensation(motorBuilder.getNominalVoltage()),
        String.format(
            "[MotorConfig.configureMotor]: Failed to set voltage compensation to %s.",
            motorBuilder.getNominalVoltage()));
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
          motor.enableSoftLimit(reverseSoftLimit.getDirection(), true),
          "[MotorConfig.configureMotor]: Failed to set reverse soft limit.");
      RevUtil.checkRevError(
          motor.setSoftLimit(reverseSoftLimit.getDirection(), reverseSoftLimit.getLimit()),
          String.format(
              "[MotorConfig.configureMotor]: Failed to set reverse soft limit to %s.",
              reverseSoftLimit.getDirection()));

      message +=
          String.format(
              "Reverse soft limit -> true\n\t\tReverse soft limit direction -> %s\n\t\tReverse soft limit -> %s\n\t",
              reverseSoftLimit.getDirection(), reverseSoftLimit.getLimit());
    }

    if (motorBuilder.getForwardSoftLimit() != null) {
      MotorSoftLimit forwardSoftLimit = motorBuilder.getForwardSoftLimit();
      RevUtil.checkRevError(
          motor.enableSoftLimit(forwardSoftLimit.getDirection(), true),
          "[MotorConfig.configureMotor]: Failed to set forward soft limit.");
      RevUtil.checkRevError(
          motor.setSoftLimit(forwardSoftLimit.getDirection(), forwardSoftLimit.getLimit()),
          String.format(
              "[MotorConfig.configureMotor]: Failed to set forward soft limit to %s.",
              forwardSoftLimit.getDirection()));

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
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureEncoder() {
    return configureEncoder(0);
  }

  /**
   * Configures the motor encoder parameters, including initial position, feedback device, and
   * conversion factors.
   *
   * @param initialPosition The initial position for the motor encoder.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureEncoder(double initialPosition) {
    // TODO: Refactor

    String message =
        String.format(
            "[MotorConfig.configureEncoder] %s - %s:\n\t",
            motorBuilder.getName(), motorEncoder.getClass());

    if (motorEncoder instanceof RelativeEncoder) {
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder).setPosition(initialPosition),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set motor encoder position to %s.",
              initialPosition));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set position conversion factor to %s.",
              motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set velocity conversion factor to %s.",
              motorBuilder.getVelocityConversionFactor()));
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      RevUtil.checkRevError(
          ((AbsoluteEncoder) motorEncoder).setInverted(motorBuilder.isEncoderInverted()));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set position conversion factor to %s.",
              motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set velocity conversion factor to %s.",
              motorBuilder.getVelocityConversionFactor()));
    } else if (motorEncoder instanceof SparkAbsoluteEncoder) {
      ((SparkAbsoluteEncoder) motorEncoder).setInverted(motorBuilder.isEncoderInverted());
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set position conversion factor to %s.",
              motorBuilder.getPositionConversionFactor()));
      RevUtil.checkRevError(
          ((RelativeEncoder) motorEncoder)
              .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()),
          String.format(
              "[MotorCongig.configureEncoder]: Failed to set velocity conversion factor to %s.",
              motorBuilder.getVelocityConversionFactor()));
    }

    message +=
        String.format(
            "Inital position -> %s\n\tPosition conversion factor -> %s\n\tVelocity conversion factor -> %s\n\t",
            initialPosition,
            motorBuilder.getPositionConversionFactor(),
            motorBuilder.getVelocityConversionFactor());

    if (motor.getPIDController() != null) {
      message += String.format("Set feedback device -> %s\n", motorEncoder.getClass());
      RevUtil.checkRevError(
          motor.getPIDController().setFeedbackDevice(motorEncoder),
          "[MotorCongig.configureEncoder]: Failed to set PID feedback device.");
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
            motorBuilder.getName(),
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

    RevUtil.checkRevError(
        motorPIDController.setP(motorPid.getP(), slot),
        String.format("[MotorConfig.configurePid]: Failed to set P to %s", motorPid.getP()));

    RevUtil.checkRevError(
        motorPIDController.setI(motorPid.getI(), slot),
        String.format("[MotorConfig.configurePid]: Failed to set I to %s", motorPid.getI()));

    RevUtil.checkRevError(
        motorPIDController.setIZone(motorPid.getIZone(), slot),
        String.format(
            "[MotorConfig.configurePid]: Failed to set I zone to %s", motorPid.getIZone()));

    RevUtil.checkRevError(
        motorPIDController.setD(motorPid.getD(), slot),
        String.format("[MotorConfig.configurePid]: Failed to set D to %s", motorPid.getD()));

    RevUtil.checkRevError(
        motorPIDController.setFF(motorPid.getFf(), slot),
        String.format("[MotorConfig.configurePid]: Failed to set FF to %s", motorPid.getFf()));

    RevUtil.checkRevError(
        motorPIDController.setOutputRange(motorPid.getMinOutput(), motorPid.getMaxOutput(), slot),
        String.format(
            "[MotorConfig.configurePid]: Failed to set output range to [%s, %s]",
            motorPid.getMinOutput(), motorPid.getMaxOutput()));

    if (motorPid.isPositionPidWrappingEnabled()) {
      configurePositionalPidWrapping(motorPid);
    }

    String message =
        String.format(
            "[MotorConfig.configurePid] %s:\n\tSet P -> %s\n\tSet I -> %s\n\tSet I Zone -> %s\n\tSet D -> %s\n\tSet FF -> %s\n\tSet min output -> %s\n\tSet max output -> %s\n",
            motorBuilder.getName(),
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
        motorPIDController.setPositionPIDWrappingEnabled(positionPidWrappingEnabled),
        "[MotorConfig.configurePositionalPidWrapping]: Failed to enable position PID wrapping.");
    RevUtil.checkRevError(
        motorPIDController.setPositionPIDWrappingMinInput(motorPid.getPositionPidWrappingMin()),
        String.format(
            "[MotorConfig.configurePositionalPidWrapping]: Failed to enable position PID min input to %s.",
            motorPid.getPositionPidWrappingMin()));
    RevUtil.checkRevError(
        motorPIDController.setPositionPIDWrappingMaxInput(motorPid.getPositionPidWrappingMax()),
        String.format(
            "[MotorConfig.configurePositionalPidWrapping]: Failed to enable position PID max input to %s.",
            motorPid.getPositionPidWrappingMax()));

    String message =
        String.format(
            "[MotorConfig.configurePositionalPidWrapping] %s:\n\tSet positional PID wrapping enabled -> true\n\tSet positional PID wrapping min input -> %s\n\tSet positional PID wrapping max input -> %s\n",
            motorBuilder.getName(),
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
    RevUtil.checkRevError(
        motor.follow(leaderMotor, motorBuilder.isFollowInverted()),
        String.format(
            "[MotorConfig.follow]: Failed to follow motor %s.", leaderMotor.getDeviceId()));

    String message =
        String.format(
            "[MotorConfig.follow] %s:\n\tSet follow -> true\n\tSet leader (ID) -> %s\n\tSet follow inverted -> %s\n",
            motorBuilder.getName(), leaderMotor.getDeviceId(), motorBuilder.isFollowInverted());
    logEntry.append(message);

    return this;
  }

  /** Burns parameters onto the motor flash. */
  public void burnFlash() {
    String message =
        String.format(
            "[MotorConfig.burnFlash] %s:\n\tBurn flash delay (seconds) -> 0.25\n",
            motorBuilder.getName(), 0.25);
    logEntry.append(message);

    Timer.delay(0.25);
    RevUtil.checkRevError(motor.burnFlash(), "[MotorConfig.burnFlash]: Failed to burn flash.");
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

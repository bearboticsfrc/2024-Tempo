package frc.bearbotics.motor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.bearbotics.motor.cancoder.CANCoders;
import frc.robot.util.RevUtil;

/** Configuration class for motor parameters, including feedback sensors and builders. */
public class MotorConfig {

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
   * Configures generic motor parameters, including inversion, idle mode, voltage compensation, and
   * current limit.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureMotor() {
    motor.setInverted(motorBuilder.isMotorInverted());

    String motorDescription =
        motorBuilder.getName() == null
            ? motorBuilder.getModuleName()
            : String.format("%s - %s", motorBuilder.getModuleName(), motorBuilder.getName());

    RevUtil.checkRevError(motor.setIdleMode(motorBuilder.getIdleMode()));
    RevUtil.checkRevError(motor.enableVoltageCompensation(motorBuilder.getNominalVoltage()));
    RevUtil.checkRevError(motor.setSmartCurrentLimit(motorBuilder.getCurrentLimit()));
    RevUtil.setPeriodicFramePeriodHigh(motor, motorDescription);

    if (motorBuilder.getReverseSoftLimit() != null) {
      MotorSoftLimit reverseSoftLimit = motorBuilder.getReverseSoftLimit();
      RevUtil.checkRevError(
          motor.setSoftLimit(reverseSoftLimit.getDirection(), reverseSoftLimit.getLimit()));
    }

    if (motorBuilder.getForwardSoftLimit() != null) {
      MotorSoftLimit forwardSoftLimit = motorBuilder.getForwardSoftLimit();
      RevUtil.checkRevError(
          motor.setSoftLimit(forwardSoftLimit.getDirection(), forwardSoftLimit.getLimit()));
    }

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
    if (motorEncoder instanceof RelativeEncoder) {
      ((RelativeEncoder) motorEncoder).setPosition(initialPosition.getRadians());
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      RevUtil.checkRevError(
          ((AbsoluteEncoder) motorEncoder).setInverted(motorBuilder.isEncoderInverted()));
    }

    if (motor.getPIDController() != null) {
      RevUtil.checkRevError(motor.getPIDController().setFeedbackDevice(motorEncoder));
    }

    // Since both relative and absolute encoders
    // define this method, the cast shouldn't matter.
    RevUtil.checkRevError(
        ((RelativeEncoder) motorEncoder)
            .setPositionConversionFactor(motorBuilder.getPositionConversionFactor()));
    RevUtil.checkRevError(
        ((RelativeEncoder) motorEncoder)
            .setVelocityConversionFactor(motorBuilder.getVelocityConversionFactor()));

    return this;
  }

  /**
   * Configures the absolute encoder parameters using CANCoders.
   *
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configureAbsoluteEncoder() {
    CANCoders.getInstance().configure(motorBuilder.getCanCoderBuilder());
    return this;
  }

  /**
   * Configures the PID parameters for the motor.
   *
   * @param motorPid The MotorPidBuilder containing PID parameters.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configurePID(MotorPidBuilder motorPid) {
    return configurePID(motorPid, 0);
  }

  /**
   * Configures the PID parameters for the motor in a specific slot.
   *
   * @param motorPid The MotorPidBuilder containing PID parameters.
   * @param slot The PID slot to configure.
   * @return This MotorConfig instance for method chaining.
   */
  public MotorConfig configurePID(MotorPidBuilder motorPid, int slot) {
    SparkPIDController motorPIDController = motor.getPIDController();
    RevUtil.checkRevError(motorPIDController.setP(motorPid.getP(), slot));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getI(), slot));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getIZone(), slot));
    RevUtil.checkRevError(motorPIDController.setD(motorPid.getD(), slot));
    RevUtil.checkRevError(motorPIDController.setFF(motorPid.getFf(), slot));
    RevUtil.checkRevError(
        motorPIDController.setOutputRange(motorPid.getMinOutput(), motorPid.getMaxOutput(), slot));
    configurePositionalPidWrapping(motorPid);

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

    if (positionPidWrappingEnabled) {
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingEnabled(positionPidWrappingEnabled));
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingMinInput(motorPid.getPositionPidWrappingMin()));
      RevUtil.checkRevError(
          motorPIDController.setPositionPIDWrappingMaxInput(motorPid.getPositionPidWrappingMax()));
    }

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
    return this;
  }

  /** Burns parameters onto the motor flash. */
  public void burnFlash() {
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

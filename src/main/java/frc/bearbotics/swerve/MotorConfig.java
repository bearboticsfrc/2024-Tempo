package frc.bearbotics.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.bearbotics.swerve.cancoder.CANCoders;
import frc.bearbotics.swerve.cancoder.CANCoders.CANCoderBuilder;
import frc.robot.util.RevUtil;

public class MotorConfig {
  private CANSparkBase motor;
  private MotorFeedbackSensor motorEncoder;
  private CANCoderBuilder canCoderBuilder;
  private String moduleName;
  private String name;
  private boolean motorInverted;
  private boolean encoderInverted;
  private double nominalVoltage;
  private int currentLimit;
  private double positionConversionFactor;
  private double velocityConversionFactor;
  private IdleMode idleMode;
  private boolean followInverted;

  /**
   * @param motor Specific motor to configure
   * @param motorEncoder Motor encoder for this motor
   * @param inverted Whether this motor should be inverted or not
   * @param nominalVoltage The nominal voltage compansation for this motor
   * @param currentLimit The current limit for this motor
   */
  public MotorConfig(
      CANSparkBase motor,
      MotorFeedbackSensor motorEncoder,
      CANCoderBuilder canCoderBuilder,
      String moduleName,
      String name,
      boolean motorInverted,
      boolean encoderInverted,
      double nominalVoltage,
      int currentLimit,
      double positionConversionFactor,
      double velocityConversionFactor,
      IdleMode idleMode,
      boolean followInverted) {
    this(
        motor,
        motorEncoder,
        moduleName,
        name,
        motorInverted,
        encoderInverted,
        nominalVoltage,
        currentLimit,
        positionConversionFactor,
        velocityConversionFactor,
        idleMode,
        followInverted);
    this.canCoderBuilder = canCoderBuilder;
  }

  /**
   * @param motor Specific motor to configure
   * @param motorEncoder Motor encoder for this motor
   * @param inverted Whether this motor should be inverted or not
   * @param nominalVoltage The nominal voltage compansation for this motor
   * @param currentLimit The current limit for this motor
   */
  public MotorConfig(
      CANSparkBase motor,
      MotorFeedbackSensor motorEncoder,
      String moduleName,
      String name,
      boolean motorInverted,
      boolean encoderInverted,
      double nominalVoltage,
      int currentLimit,
      double positionConversionFactor,
      double velocityConversionFactor,
      IdleMode idleMode,
      boolean followInverted) {
    this(
        motor,
        moduleName,
        name,
        motorInverted,
        encoderInverted,
        nominalVoltage,
        currentLimit,
        positionConversionFactor,
        velocityConversionFactor,
        idleMode,
        followInverted);
    this.motorEncoder = motorEncoder;
  }

  /**
   * @param motor Specific motor to configure
   * @param motorEncoder Motor encoder for this motor
   * @param inverted Whether this motor should be inverted or not
   * @param nominalVoltage The nominal voltage compansation for this motor
   * @param currentLimit The current limit for this motor
   */
  public MotorConfig(
      CANSparkBase motor,
      String moduleName,
      String name,
      boolean motorInverted,
      boolean encoderInverted,
      double nominalVoltage,
      int currentLimit,
      double positionConversionFactor,
      double velocityConversionFactor,
      IdleMode idleMode,
      boolean followInverted) {
    this.motor = motor;
    this.moduleName = moduleName;
    this.name = name;
    this.motorInverted = motorInverted;
    this.encoderInverted = encoderInverted;
    this.nominalVoltage = nominalVoltage;
    this.currentLimit = currentLimit;
    this.positionConversionFactor = positionConversionFactor;
    this.velocityConversionFactor = velocityConversionFactor;
    this.idleMode = idleMode;
    this.followInverted = followInverted;
  }

  /*
   * Generic configuration method
   */
  public MotorConfig configureMotor() {
    motor.setInverted(motorInverted);

    String motorDescription =
        name == null ? moduleName : String.format("%s - %s", moduleName, name);

    RevUtil.checkRevError(motor.setIdleMode(idleMode));
    RevUtil.checkRevError(motor.enableVoltageCompensation(nominalVoltage));
    RevUtil.checkRevError(motor.setSmartCurrentLimit(currentLimit));
    RevUtil.setPeriodicFramePeriodHigh(motor, motorDescription);

    return this;
  }

  public MotorConfig configureEncoder(Rotation2d initalPosition) {
    if (motorEncoder instanceof RelativeEncoder) {
      ((RelativeEncoder) motorEncoder).setPosition(initalPosition.getRadians());
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      RevUtil.checkRevError(((AbsoluteEncoder) motorEncoder).setInverted(encoderInverted));
    }

    if (motor.getPIDController() != null) {
      RevUtil.checkRevError(motor.getPIDController().setFeedbackDevice(motorEncoder));
    }

    // Since both relative and absolute encoders
    // define this method, the cast shouldn't matter.
    RevUtil.checkRevError(
        ((RelativeEncoder) motorEncoder).setPositionConversionFactor(positionConversionFactor));
    RevUtil.checkRevError(
        ((RelativeEncoder) motorEncoder).setVelocityConversionFactor(velocityConversionFactor));

    return this;
  }

  public MotorConfig configureAbsoluteEncoder() {
    CANCoders.getInstance().configure(canCoderBuilder);

    if (!CANCoders.getInstance().isInitalized(canCoderBuilder.getId())) {
      Timer.delay(0.1);
    }
    return this;
  }

  public MotorConfig configurePID(MotorPidBuilder motorPid) {
    configurePID(motorPid, 0);
    return this;
  }

  public MotorConfig configurePID(MotorPidBuilder motorPid, int slot) {
    SparkPIDController motorPIDController = motor.getPIDController();
    RevUtil.checkRevError(motorPIDController.setP(motorPid.getP(), slot));
    RevUtil.checkRevError(motorPIDController.setI(motorPid.getI(), slot));
    RevUtil.checkRevError(motorPIDController.setD(motorPid.getD(), slot));
    RevUtil.checkRevError(motorPIDController.setFF(motorPid.getFf(), slot));
    RevUtil.checkRevError(
        motorPIDController.setOutputRange(motorPid.getMinOutput(), motorPid.getMaxOutput(), slot));
    configurePositionalPidWrapping(motorPid);

    return this;
  }

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

  public MotorConfig follow(CANSparkBase leaderMotor) {
    motor.follow(leaderMotor, followInverted);
    return this;
  }

  public void burnFlash() {
    Timer.delay(0.25);
    RevUtil.checkRevError(motor.burnFlash());
    Timer.delay(0.25);
    // Burn settings onto motor flash
    // might not work, needs a delay after setting values
  }

  public static MotorConfig fromMotorConstants(
      CANSparkBase motor, MotorFeedbackSensor motorEncoder, MotorBuilder constants) {
    return new MotorConfig(
        motor,
        motorEncoder,
        constants.getModuleName(),
        constants.getName(),
        constants.isMotorInverted(),
        constants.isEncoderInverted(),
        constants.getNominalVoltage(),
        constants.getCurrentLimit(),
        constants.getPositionConversionFactor(),
        constants.getVelocityConversionFactor(),
        constants.getIdleMode(),
        constants.isFollowInverted());
  }

  public static MotorConfig fromMotorConstants(CANSparkBase motor, MotorBuilder constants) {
    return new MotorConfig(
        motor,
        constants.getModuleName(),
        constants.getName(),
        constants.isMotorInverted(),
        constants.isEncoderInverted(),
        constants.getNominalVoltage(),
        constants.getCurrentLimit(),
        constants.getPositionConversionFactor(),
        constants.getVelocityConversionFactor(),
        constants.getIdleMode(),
        constants.isFollowInverted());
  }

  public static MotorConfig fromMotorConstants(
      CANSparkBase motor,
      MotorFeedbackSensor motorEncoder,
      CANCoderBuilder canCoderBuilder,
      MotorBuilder constants) {
    return new MotorConfig(
        motor,
        motorEncoder,
        canCoderBuilder,
        constants.getModuleName(),
        constants.getName(),
        constants.isMotorInverted(),
        constants.isEncoderInverted(),
        constants.getNominalVoltage(),
        constants.getCurrentLimit(),
        constants.getPositionConversionFactor(),
        constants.getVelocityConversionFactor(),
        constants.getIdleMode(),
        constants.isFollowInverted());
  }
}

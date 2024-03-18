package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.math.QuadraticCurveInterpolator;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.bearbotics.motor.MotorSoftLimit;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ArmConstants;
import frc.robot.util.RevUtil;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkMax armMotor;

  private SparkAbsoluteEncoder armAbsoluteMotorEncoder;
  private RelativeEncoder armRelativeEncoder;

  private DigitalInput armLimitSwtich = new DigitalInput(ArmConstants.LIMIT_SWITCH_CHANNEL);

  private SparkPIDController armPidController;
  private ArmFeedforward armFeedforward;

  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(ArmConstants.Motor.TrapezoidProfile.constraints);
  private TrapezoidProfile.State targetState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);

  private final QuadraticCurveInterpolator shootAngleInterpolator =
      new QuadraticCurveInterpolator(ArmConstants.SHOOT_ANGLE_MAP);

  /**
   * Constructor for the ArmSubsystem class. Initializes the motors, encoders, and sets up the
   * ShuffleboardTab.
   */
  public ArmSubsystem() {
    setupMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.ARM_SYSTEM_TAB);
    }
  }

  /** Initializes the arm motors. */
  private void setupMotors() {
    MotorPidBuilder armMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ArmConstants.Motor.MotorPid.P)
            .withD(ArmConstants.Motor.MotorPid.D)
            .withMinOutput(ArmConstants.Motor.MotorPid.MIN_OUTPUT)
            .withMaxOutput(ArmConstants.Motor.MotorPid.MAX_OUTPUT)
            .withPositionPidWrappingEnabled(ArmConstants.Motor.MotorPid.POSITION_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(ArmConstants.Motor.MotorPid.POSITION_WRAPPING_MIN)
            .withPositionPidWrappingMax(ArmConstants.Motor.MotorPid.POSITION_WRAPPING_MAX);

    MotorSoftLimit forwardSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kForward)
            .withLimit(ArmConstants.Motor.FORWARD_SOFT_LIMIT);

    MotorBuilder armAbsoluteEncoderMotorConfig =
        new MotorBuilder()
            .withName(ArmConstants.Motor.NAME)
            .withMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ArmConstants.Motor.INVERTED)
            .withCurrentLimit(ArmConstants.Motor.CURRENT_LIMIT)
            .withNominalVoltage(ArmConstants.Motor.NOMINAL_VOLTAGE)
            .withPositionConversionFactor(
                ArmConstants.Motor.ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(
                ArmConstants.Motor.ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR)
            .withIdleMode(ArmConstants.Motor.IDLE_MODE)
            .withForwardSoftLimit(forwardSoftLimit)
            .withMotorPid(armMotorPidBuilder);

    MotorBuilder armRelativeEncoderMotorConfig =
        new MotorBuilder()
            .withPositionConversionFactor(
                ArmConstants.Motor.RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(
                ArmConstants.Motor.RELATIVE_ENCODER_VELOCITY_CONVERSION_FACTOR);

    armMotor =
        new CANSparkMax(ArmConstants.Motor.MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);

    armAbsoluteMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armRelativeEncoder = armMotor.getEncoder();

    armFeedforward =
        new ArmFeedforward(
            ArmConstants.Motor.FeedForward.STATIC,
            ArmConstants.Motor.FeedForward.GRAVITY,
            ArmConstants.Motor.FeedForward.VELOCITY);
    armPidController = armMotor.getPIDController();
    RevUtil.checkRevError(
        armRelativeEncoder.setPositionConversionFactor(
            ArmConstants.Motor.RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR));
    armRelativeEncoder.setPosition(armAbsoluteMotorEncoder.getPosition());

    MotorConfig.fromMotorConstants(armMotor, armAbsoluteMotorEncoder, armAbsoluteEncoderMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder();

    MotorConfig.fromMotorConstants(armMotor, armRelativeEncoder, armRelativeEncoderMotorConfig)
        .configureEncoder(armAbsoluteMotorEncoder.getPosition())
        .burnFlash();
  }

  /**
   * Sets up the ShuffleboardTab for displaying arm-related data on the Shuffleboard dashboard.
   *
   * @param shuffleboardTab The ShuffleboardTab instance to which arm data will be added.
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Arm Abs Pos", armAbsoluteMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Arm Rel Pos", armRelativeEncoder::getPosition);

    shuffleboardTab.addDouble("Arm Goal", () -> targetState.position);
    shuffleboardTab.addDouble("Arm Current Pos", () -> currentState.position);
    shuffleboardTab.addDouble("Arm Current Vel", () -> currentState.velocity);

    shuffleboardTab.addDouble("Arm Amps", armMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Arm Temp", armMotor::getMotorTemperature);

    shuffleboardTab.addBoolean("Is Arm Home", this::isArmHome);
    shuffleboardTab.addBoolean("Is Arm Setpoint", this::atTargetSetpoint);
  }

  private Rotation2d getPosition() {
    return Rotation2d.fromDegrees(armRelativeEncoder.getPosition());
  }
  /** Updates the arm's state periodically, ensuring smooth motion using a trapezoidal profile. */
  @Override
  public void periodic() {
    updateState();

    if (isArmHome() && targetState.position == 0) {
      armMotor.stopMotor(); // Prevent arm from pulling current when resting.
    }
  }

  /** Updates the arm's state based on the trapezoidal profile, adjusting the motor controller. */
  private void updateState() {
    currentState = trapezoidProfile.calculate(RobotConstants.CYCLE_TIME, currentState, targetState);

    armPidController.setReference(
        currentState.position,
        ControlType.kPosition,
        0,
        armFeedforward.calculate(Math.toRadians(currentState.position), currentState.velocity));
  }

  /**
   * Checks if the arm is at its home position based on the limit switch.
   *
   * @return True if the arm is at its home position, otherwise false.
   */
  public boolean isArmHome() {
    return armLimitSwtich.get();
  }

  /**
   * Checks if the arm is at the target setpoint within a specified position tolerance.
   *
   * @return True if the arm is at the target setpoint, otherwise false.
   */
  public boolean atTargetSetpoint() {
    return Math.abs(targetState.position - getPosition().getDegrees())
        < ArmConstants.POSITION_TOLERANCE;
  }

  /** Stops the arm motor. */
  public void stop() {
    armMotor.stopMotor();
  }

  /**
   * Sets the arm motor to the specified position using the ArmPosition enum.
   *
   * @param position The desired arm position.
   */
  public void set(ArmPosition position) {
    set(position.getAngle().getDegrees());
  }

  /**
   * Sets the arm motor to the specified position returned from a distance supplier.
   *
   * @param distanceSupplier A supplier providing the distance to the target.
   */
  public void set(DoubleSupplier distanceSupplier) {
    set(getPositionFromDistance(distanceSupplier.getAsDouble()));
  }

  /**
   * Sets the arm motor to the specified position in degrees.
   *
   * @param position The desired arm position in degrees.
   */
  private void set(double position) {
    targetState.position = position;
    currentState = calculateTrapezoidState(targetState);

    armPidController.setReference(
        currentState.position, ControlType.kPosition, 0, calculateFeedForward());
  }

  /**
   * Calculates the arm position from a given distance using an interpolator.
   *
   * @param distance The distance to the target.
   * @return The corresponding arm position.
   */
  private double getPositionFromDistance(double distance) {
    return shootAngleInterpolator.calculate(Math.min(distance, ArmConstants.MAX_DISTANCE));
  }

  /**
   * Calculates the trapezoidal profile state for a given target state.
   *
   * @param targetState The target state for which the profile state will be calculated.
   * @return The calculated trapezoidal profile state.
   */
  private TrapezoidProfile.State calculateTrapezoidState(TrapezoidProfile.State targetState) {
    return trapezoidProfile.calculate(
        RobotConstants.CYCLE_TIME,
        new TrapezoidProfile.State(getPosition().getDegrees(), currentState.velocity),
        targetState);
  }

  /**
   * Calculates the feedforward value for the arm motor based on the current position and velocity.
   *
   * @return The calculated feedforward value.
   */
  private double calculateFeedForward() {
    return armFeedforward.calculate(getPosition().getRadians(), currentState.velocity);
  }

  /** Enum representing different positions of the arm. */
  public enum ArmPosition {
    HOME(Rotation2d.fromDegrees(0)),
    LINE_SHOOT(Rotation2d.fromDegrees(18)),
    PODIUM_SHOOT(Rotation2d.fromDegrees(28)),
    AMP_SHOOT(Rotation2d.fromDegrees(80)),
    STAGE_SHOOT(Rotation2d.fromDegrees(36)),
    SHOOT(Rotation2d.fromDegrees(90));

    private final Rotation2d angle;

    /**
     * Constructor for ArmPosition.
     *
     * @param angle The rotation angle associated with the position.
     */
    private ArmPosition(Rotation2d angle) {
      this.angle = angle;
    }

    /**
     * Get the rotation angle associated with the position.
     *
     * @return The rotation angle.
     */
    public Rotation2d getAngle() {
      return angle;
    }
  }
}

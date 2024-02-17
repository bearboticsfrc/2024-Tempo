package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.bearbotics.motor.MotorSoftLimit;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;

  private SparkAbsoluteEncoder armMotorEncoder;

  private ArmFeedforward armFeedforward;

  private DigitalInput armLimitSwtich = new DigitalInput(ArmConstants.LIMIT_SWITCH_CHANNEL);

  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(ArmConstants.Motor.TrapezoidProfile.constraints);
  private TrapezoidProfile.State targetState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);

  public ArmSubsystem() {
    setupMotors();
    setupShuffleboardTab(RobotConstants.ARM_SYSTEM_TAB);
  }

  private void setupMotors() {
    MotorPidBuilder armMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ArmConstants.Motor.MotorPid.P)
            .withMinOutput(ArmConstants.Motor.MotorPid.MIN_OUTPUT);

    MotorSoftLimit forwardSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kForward)
            .withLimit(ArmConstants.Motor.FORWARD_SOFT_LIMIT);

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .withName(ArmConstants.Motor.NAME)
            .withMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ArmConstants.Motor.INVERTED)
            .withCurrentLimit(ArmConstants.Motor.CURRENT_LIMIT)
            .withPositionConversionFactor(ArmConstants.Motor.POSITION_CONVERSION_FACTOR)
            .withIdleMode(ArmConstants.Motor.IDLE_MODE)
            .withForwardSoftLimit(forwardSoftLimit)
            .withMotorPid(armMotorPidBuilder);

    armFeedforward =
        new ArmFeedforward(
            ArmConstants.Motor.FeedForward.STATIC,
            ArmConstants.Motor.FeedForward.GRAVITY,
            ArmConstants.Motor.FeedForward.VELOCITY);

    armMotor =
        new CANSparkMax(armMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    MotorConfig.fromMotorConstants(armMotor, armMotorEncoder, armMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder(Rotation2d.fromDegrees(0))
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Arm Pos", armMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Arm Cur", armMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Arm Temp", armMotor::getMotorTemperature);
    shuffleboardTab.addBoolean("Is Arm Home", this::isArmHome);
    shuffleboardTab.addBoolean("Is Arm Setpoint", this::atTargetSetpoint);
  }

  @Override
  public void periodic() {
    if (targetState.position == 0 && isArmHome()) {
      armMotor.stopMotor(); // Prevent arm from pulling current when resting.
    }

    currentState = trapezoidProfile.calculate(RobotConstants.CYCLE_TIME, currentState, targetState);

    armMotor
        .getPIDController()
        .setReference(
            currentState.position,
            ControlType.kPosition,
            0,
            armFeedforward.calculate(getPosition().getRadians(), currentState.velocity));
  }

  public boolean isArmHome() {
    return armLimitSwtich.get();
  }

  public boolean atTargetSetpoint() {
    return Math.abs(targetState.position - getPosition().getDegrees())
        < ArmConstants.POSITION_TOLERANCE;
  }

  private Rotation2d getPosition() {
    return Rotation2d.fromDegrees(armMotorEncoder.getPosition());
  }

  /**
   * Set the arm motor to the specified position.
   *
   * @param position The desired arm position.
   */
  public void set(double position) {
    targetState.position = position;
    currentState =
        trapezoidProfile.calculate(
            RobotConstants.CYCLE_TIME,
            new TrapezoidProfile.State(getPosition().getDegrees(), armMotorEncoder.getVelocity()),
            targetState);

    double feedForward =
        armFeedforward.calculate(getPosition().getRotations(), armMotorEncoder.getVelocity());

    armMotor.getPIDController().setReference(position, ControlType.kPosition, 0, feedForward);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  /** Enum representing different positions of the arm. */
  public enum ArmPosition {
    HOME(Rotation2d.fromDegrees(0)),
    AMP_SHOOT(Rotation2d.fromDegrees(40)),
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

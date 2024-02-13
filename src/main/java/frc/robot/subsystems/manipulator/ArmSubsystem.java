package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.bearbotics.motor.MotorSoftLimit;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollower;

  private SparkAbsoluteEncoder armMotorEncoder;

  private ArmFeedforward armFeedforward;

  public ArmSubsystem() {
    setupMotors();
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void setupMotors() {
    MotorPidBuilder armMotorLowerPid =
        new MotorPidBuilder()
            .withP(ArmConstants.Motor.MotorLowerPid.P)
            .withMinOutput(ArmConstants.Motor.MotorLowerPid.MIN_OUTPUT)
            .withMaxOutput(ArmConstants.Motor.MotorLowerPid.MAX_OUTPUT);

    MotorPidBuilder armMotorRaisePid =
        new MotorPidBuilder()
            .withP(ArmConstants.Motor.MotorRaisePid.P)
            .withMinOutput(ArmConstants.Motor.MotorRaisePid.MIN_OUTPUT)
            .withMaxOutput(ArmConstants.Motor.MotorRaisePid.MAX_OUTPUT);

    MotorSoftLimit forwardSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kForward)
            .withLimit(Rotation2d.fromDegrees(81));

    MotorSoftLimit reverseSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kReverse)
            .withLimit(Rotation2d.fromDegrees(0));

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .withModuleName(ArmConstants.Motor.MODULE_NAME)
            .withMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ArmConstants.Motor.INVERTED)
            .withCurrentLimit(ArmConstants.Motor.CURRENT_LIMT)
            .withReverseSoftLimit(forwardSoftLimit)
            .withForwardSoftLimit(reverseSoftLimit)
            .withMotorPid(armMotorLowerPid, 0)
            .withMotorPid(armMotorRaisePid, 1);

    MotorBuilder armMotorFollowerConfig =
        new MotorBuilder()
            .withModuleName(ArmConstants.MotorFollower.MODULE_NAME)
            .withMotorPort(ArmConstants.MotorFollower.MOTOR_PORT)
            .withMotorInverted(ArmConstants.MotorFollower.FOLLOW_INVERTED)
            .withCurrentLimit(ArmConstants.MotorFollower.CURRENT_LIMT)
            .withReverseSoftLimit(forwardSoftLimit)
            .withForwardSoftLimit(reverseSoftLimit);

    armMotor =
        new CANSparkMax(armMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    armMotorFollower =
        new CANSparkMax(
            armMotorFollowerConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    armFeedforward =
        new ArmFeedforward(
            ArmConstants.Motor.FeedForward.STATIC,
            ArmConstants.Motor.FeedForward.GRAVITY,
            ArmConstants.Motor.FeedForward.VELOCITY);

    MotorConfig.fromMotorConstants(armMotor, armMotorEncoder, armMotorConfig)
        .configureMotor()
        .configureEncoder(Rotation2d.fromDegrees(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(armMotorFollower, armMotorFollowerConfig)
        .configureMotor()
        .follow(armMotor)
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Arm Pos", armMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Arm Cur", armMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Arm Temp", armMotor::getMotorTemperature);
  }

  private Rotation2d getShootAngle() {
    return new Rotation2d(); // TODO: impl
  }

  /**
   * Set the arm motor to the specified position.
   *
   * @param position The desired arm position.
   */
  public void set(ArmPosition position) {
    armMotor
        .getPIDController()
        .setReference(
            position == ArmPosition.SHOOT
                ? getShootAngle().getDegrees()
                : position.getAngle().getDegrees(),
            ControlType.kPosition,
            position.getSlot());
  }

  /** Enum representing different positions of the arm. */
  public enum ArmPosition {
    HOME(Rotation2d.fromDegrees(0), 0),
    AMP_SHOOT(Rotation2d.fromDegrees(40), 1),
    SHOOT(Rotation2d.fromDegrees(90), 1);

    private final Rotation2d angle;
    private final int slot;

    /**
     * Constructor for ArmPosition.
     *
     * @param angle The rotation angle associated with the position.
     * @param slot The PID slot associated with the position.
     */
    private ArmPosition(Rotation2d angle, int slot) {
      this.angle = angle;
      this.slot = slot;
    }

    /**
     * Get the rotation angle associated with the position.
     *
     * @return The rotation angle.
     */
    public Rotation2d getAngle() {
      return angle;
    }

    /**
     * Get the PID slot associated with the position.
     *
     * @return The PID slot.
     */
    public int getSlot() {
      return slot;
    }
  }
}

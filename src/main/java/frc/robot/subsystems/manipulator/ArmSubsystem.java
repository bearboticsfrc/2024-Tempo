package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
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

  private GenericEntry debug_setPoint =
      DriveConstants.MANIPULATOR_SYSTEM_TAB.add("Arm Set Point", this.setPoint).getEntry();

  private GenericEntry debug_FfG =
      DriveConstants.MANIPULATOR_SYSTEM_TAB.add("Arm FF G", this.ffG).getEntry();

  private GenericEntry debug_P =
      DriveConstants.MANIPULATOR_SYSTEM_TAB.add("Arm P", this.p).getEntry();

  private GenericEntry debug_D =
      DriveConstants.MANIPULATOR_SYSTEM_TAB.add("Arm D", this.d).getEntry();

  private double setPoint = 0;
  private double ffG = 0;
  private double p = 0;
  private double d = 0;

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
        new MotorSoftLimit().withDirection(SoftLimitDirection.kForward).withLimit(81);

    MotorSoftLimit reverseSoftLimit =
        new MotorSoftLimit().withDirection(SoftLimitDirection.kReverse).withLimit(0);

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .withName(ArmConstants.Motor.MODULE_NAME)
            .withMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ArmConstants.Motor.INVERTED)
            .withCurrentLimit(ArmConstants.Motor.CURRENT_LIMT)
            .withReverseSoftLimit(forwardSoftLimit)
            .withForwardSoftLimit(reverseSoftLimit)
            .withMotorPid(armMotorLowerPid, 0)
            .withMotorPid(armMotorRaisePid, 1);

    MotorBuilder armMotorFollowerConfig =
        new MotorBuilder()
            .withName(ArmConstants.MotorFollower.MODULE_NAME)
            .withMotorPort(ArmConstants.MotorFollower.MOTOR_PORT)
            .withFollowInverted(ArmConstants.MotorFollower.FOLLOW_INVERTED)
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
    shuffleboardTab.addDouble("Follower Arm Cur", armMotorFollower::getOutputCurrent);
  }

  private Rotation2d getShootAngle() {
    return new Rotation2d(); // TODO: impl
  }

  @Override
  public void periodic() {
    if (debug_D.getDouble(d) == d
        && debug_P.getDouble(p) == p
        && debug_FfG.getDouble(ffG) == ffG
        && debug_setPoint.getDouble(setPoint) == setPoint) {
      return;
    }

    MotorPidBuilder armMotorPid =
        new MotorPidBuilder()
            .withP(debug_P.getDouble(ArmConstants.Motor.MotorLowerPid.P))
            .withD(debug_D.getDouble(ArmConstants.Motor.MotorLowerPid.D))
            .withMinOutput(ArmConstants.Motor.MotorLowerPid.MIN_OUTPUT)
            .withMaxOutput(ArmConstants.Motor.MotorLowerPid.MAX_OUTPUT);

    armFeedforward =
        new ArmFeedforward(
            ArmConstants.Motor.FeedForward.STATIC,
            debug_FfG.getDouble(ArmConstants.Motor.FeedForward.GRAVITY),
            ArmConstants.Motor.FeedForward.VELOCITY);

    MotorBuilder armMotorConfig = new MotorBuilder().withMotorPid(armMotorPid);

    MotorConfig.fromMotorConstants(armMotor, armMotorEncoder, armMotorConfig).configurePid();

    armMotor
        .getPIDController()
        .setReference(debug_setPoint.getDouble(setPoint), ControlType.kPosition);
  }
  /**
   * Set the arm motor to the specified position.
   *
   * @param position The desired arm position.
   */
  public void set(ArmPosition position) {
    armMotor
        .getPIDController()
        .setReference(debug_setPoint.getDouble(setPoint), ControlType.kPosition);
  }

  public void stop() {
    armMotor.stopMotor();
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

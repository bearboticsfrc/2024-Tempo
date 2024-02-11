package frc.robot.subsystems.manipulator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.swerve.MotorConfig;
import frc.bearbotics.swerve.MotorConfig.MotorBuilder;
import frc.bearbotics.swerve.MotorConfig.MotorPIDBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollower;

  private AbsoluteEncoder armMotorEncoder;

  public ArmSubsystem() {
    setupMotors();
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void setupMotors() {
    MotorPIDBuilder armMotorLowerPid =
        new MotorPIDBuilder()
            .setP(ArmConstants.Motor.MotorLowerPid.P)
            .setFf(ArmConstants.Motor.MotorLowerPid.Ff)
            .setMinOutput(ArmConstants.Motor.MotorLowerPid.MIN_OUTPUT)
            .setMaxOutput(ArmConstants.Motor.MotorLowerPid.MAX_OUTPUT);

    MotorPIDBuilder armMotorRaisePid =
        new MotorPIDBuilder()
            .setP(ArmConstants.Motor.MotorRaisePid.P)
            .setFf(ArmConstants.Motor.MotorRaisePid.Ff)
            .setMinOutput(ArmConstants.Motor.MotorRaisePid.MIN_OUTPUT)
            .setMaxOutput(ArmConstants.Motor.MotorRaisePid.MAX_OUTPUT);

    MotorBuilder armMotorConfig =
        new MotorBuilder()
            .setModuleName(ArmConstants.Motor.MODULE_NAME)
            .setMotorPort(ArmConstants.Motor.MOTOR_PORT)
            .setMotorInverted(ArmConstants.Motor.INVERTED)
            .setCurrentLimit(ArmConstants.Motor.CURRENT_LIMT)
            .setMotorPid(armMotorLowerPid, 0)
            .setMotorPid(armMotorRaisePid, 1);

    MotorBuilder armMotorFollowerConfig =
        new MotorBuilder()
            .setModuleName(ArmConstants.MotorFollower.MODULE_NAME)
            .setMotorPort(ArmConstants.MotorFollower.MOTOR_PORT)
            .setMotorInverted(ArmConstants.MotorFollower.INVERTED)
            .setCurrentLimit(ArmConstants.MotorFollower.CURRENT_LIMT);

    armMotor =
        new CANSparkMax(armMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    armMotorFollower =
        new CANSparkMax(
            armMotorFollowerConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    MotorConfig.fromMotorConstants(armMotor, armMotorEncoder, armMotorConfig)
        .configureMotor()
        .configureEncoder(Rotation2d.fromDegrees(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(armMotorFollower, armMotorFollowerConfig)
        .configureMotor()
        .follow(armMotor, true)
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Arm Pos", armMotorEncoder::getPosition);
  }

  public Command getArmRunCommand(ArmPosition position) {
    final double angle;

    if (position == ArmPosition.SHOOT) {
      angle = getShootAngle(getDistance());
    } else {
      angle = position.getAngle().getDegrees();
    }

    return new InstantCommand(
        () ->
            armMotor
                .getPIDController()
                .setReference(angle, ControlType.kPosition, position.getSlot()));
  }

  private double getDistance() {
    return 0;
  }

  private double getShootAngle(double distance) {
    return 0;
  }

  public enum ArmPosition {
    HOME(Rotation2d.fromDegrees(0), 0),
    AMP_SHOOT(Rotation2d.fromDegrees(85), 1),
    SHOOT(Rotation2d.fromDegrees(90), 1);

    private final Rotation2d angle;
    private final int slot;

    private ArmPosition(Rotation2d angle, int slot) {
      this.angle = angle;
      this.slot = slot;
    }

    public Rotation2d getAngle() {
      return angle;
    }

    public int getSlot() {
      return slot;
    }
  }
}

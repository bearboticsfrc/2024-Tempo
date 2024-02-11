package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.swerve.MotorConfig;
import frc.bearbotics.swerve.MotorConfig.MotorBuilder;
import frc.bearbotics.swerve.MotorConfig.MotorPIDBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor;
  private CANSparkMax climberMotorFollower;

  private RelativeEncoder climberMotorEncoder;

  public ClimberSubsystem() {
    setupMotors();
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void setupMotors() {
    MotorPIDBuilder climberMotorPid =
        new MotorPIDBuilder()
            .setP(ClimberConstants.Motor.MotorPid.P)
            .setFf(ClimberConstants.Motor.MotorPid.Ff)
            .setMinOutput(ClimberConstants.Motor.MotorPid.MIN_OUTPUT)
            .setMaxOutput(ClimberConstants.Motor.MotorPid.MAX_OUTPUT);

    MotorBuilder climberMotorConfig =
        new MotorBuilder()
            .setModuleName(ClimberConstants.Motor.MODULE_NAME)
            .setMotorPort(ClimberConstants.Motor.MOTOR_PORT)
            .setMotorInverted(ClimberConstants.Motor.INVERTED)
            .setCurrentLimit(ClimberConstants.Motor.CURRENT_LIMT)
            .setMotorPID(climberMotorPid);

    MotorBuilder climberMotorFollowerConfig =
        new MotorBuilder()
            .setModuleName(ClimberConstants.MotorFollower.MODULE_NAME)
            .setMotorPort(ClimberConstants.MotorFollower.MOTOR_PORT)
            .setCurrentLimit(ClimberConstants.MotorFollower.CURRENT_LIMT);

    climberMotor =
        new CANSparkMax(climberMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    climberMotorFollower =
        new CANSparkMax(
            climberMotorFollowerConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    climberMotorEncoder = climberMotor.getEncoder();

    MotorConfig.fromMotorConstants(climberMotor, climberMotorEncoder, climberMotorConfig)
        .configureMotor()
        .configureEncoder(Rotation2d.fromDegrees(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(climberMotorFollower, climberMotorFollowerConfig)
        .configureMotor()
        .follow(climberMotor, ClimberConstants.MotorFollower.FOLLOW_INVERTED)
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Climber Pos", climberMotorEncoder::getPosition);
  }

  public void set(ClimberPosition position) {
    climberMotor.getPIDController().setReference(position.getPosition(), ControlType.kPosition);
  }

  public void set(double speed) {
    climberMotor.set(speed);
  }

  public enum ClimberPosition {
    RETRACTED(0),
    EXTENDED(106);

    private final double position;

    private ClimberPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}

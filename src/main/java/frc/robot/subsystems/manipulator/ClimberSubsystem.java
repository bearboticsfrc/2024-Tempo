package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorSoftLimit;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor;
  private CANSparkMax climberMotorFollower;

  private RelativeEncoder climberMotorEncoder;

  private DigitalInput climberLimitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_CHANNEL);

  public ClimberSubsystem() {
    setupMotors();
    setupShuffleboardTab(RobotConstants.CLIMBER_SYSTEM_TAB);
  }

  private void setupMotors() {
    MotorSoftLimit climbeSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kForward)
            .withLimit(ClimberConstants.Motor.FORWARD_SOFT_LIMIT);

    MotorBuilder climberMotorConfig =
        new MotorBuilder()
            .withName(ClimberConstants.Motor.NAME)
            .withMotorPort(ClimberConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ClimberConstants.Motor.INVERTED)
            .withCurrentLimit(ClimberConstants.Motor.CURRENT_LIMT)
            .withForwardSoftLimit(climbeSoftLimit);

    MotorBuilder climberMotorFollowerConfig =
        new MotorBuilder()
            .withName(ClimberConstants.MotorFollower.NAME)
            .withMotorPort(ClimberConstants.MotorFollower.MOTOR_PORT)
            .withCurrentLimit(ClimberConstants.MotorFollower.CURRENT_LIMT)
            .withFollowInverted(ClimberConstants.MotorFollower.FOLLOW_INVERTED);

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
        .follow(climberMotor)
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Climber Position", climberMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Climber Current", climberMotor::getOutputCurrent);
    shuffleboardTab.addBoolean("Climber Limit Switch", climberLimitSwitch::get);
  }

  @Override
  public void periodic() {
    if (isClimberHome() && climberMotorEncoder.getPosition() != 0) {
      climberMotorEncoder.setPosition(0);
    }
  }

  public boolean isClimberHome() {
    return climberLimitSwitch.get();
  }

  /**
   * Set the climber motor to the specified speed using open-loop control.
   *
   * @param speed The desired speed for the climber motor.
   */
  public void set(double speed) {
    if (isClimberHome() && speed < 0) {
      stop();
      return;
    }

    climberMotor.set(speed);
  }

  /** Stop the climber motor. */
  public void stop() {
    climberMotor.stopMotor();
  }
}

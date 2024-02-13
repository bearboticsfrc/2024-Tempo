package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
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
    MotorPidBuilder climberMotorPid =
        new MotorPidBuilder()
            .withP(ClimberConstants.Motor.MotorPid.P)
            .withFf(ClimberConstants.Motor.MotorPid.Ff)
            .withMinOutput(ClimberConstants.Motor.MotorPid.MIN_OUTPUT)
            .withMaxOutput(ClimberConstants.Motor.MotorPid.MAX_OUTPUT);

    MotorBuilder climberMotorConfig =
        new MotorBuilder()
            .withModuleName(ClimberConstants.Motor.MODULE_NAME)
            .withMotorPort(ClimberConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ClimberConstants.Motor.INVERTED)
            .withCurrentLimit(ClimberConstants.Motor.CURRENT_LIMT)
            .withMotorPID(climberMotorPid);

    MotorBuilder climberMotorFollowerConfig =
        new MotorBuilder()
            .withModuleName(ClimberConstants.MotorFollower.MODULE_NAME)
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
    shuffleboardTab.addDouble("Climber Pos", climberMotorEncoder::getPosition);
  }

  /**
   * Set the climber motor to the specified position using closed-loop control.
   *
   * @param position The desired climber position.
   */
  public void set(ClimberPosition position) {
    climberMotor.getPIDController().setReference(position.getPosition(), ControlType.kPosition);
  }

  /**
   * Set the climber motor to the specified speed using open-loop control.
   *
   * @param speed The desired speed for the climber motor.
   */
  public void set(double speed) {
    climberMotor.set(speed);
  }

  /** Enum representing different positions of the climber. */
  public enum ClimberPosition {
    RETRACTED(0),
    EXTENDED(106);

    private final double position;

    /**
     * Constructor for ClimberPosition.
     *
     * @param position The position value associated with the climber position.
     */
    private ClimberPosition(double position) {
      this.position = position;
    }

    /**
     * Get the position value associated with the climber position.
     *
     * @return The position value.
     */
    public double getPosition() {
      return position;
    }
  }
}

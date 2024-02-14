package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex shooterMotor;
  private CANSparkFlex shooterMotorFollower;

  private SparkPIDController shooterMotorPidController;

  private RelativeEncoder shooterMotorEncoder;

  private int targetVelocity;

  private Timer DEBUG_TIMER = new Timer();
  private double debug_setPointDelta;

  public ShooterSubsystem() {
    configureMotors();
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void configureMotors() {
    MotorPidBuilder shooterMotorPid =
        new MotorPidBuilder()
            .withP(ShooterConstants.Motor.MotorPid.P)
            .withI(ShooterConstants.Motor.MotorPid.I)
            .withIZone(ShooterConstants.Motor.MotorPid.I_ZONE)
            .withFf(ShooterConstants.Motor.MotorPid.Ff);

    MotorBuilder shooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.Motor.MODULE_NAME)
            .withMotorPort(ShooterConstants.Motor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.Motor.INVERTED)
            .withCurrentLimit(ShooterConstants.Motor.CURRENT_LIMT)
            .withMotorPid(shooterMotorPid)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder shooterMotorFollowerConfig =
        new MotorBuilder()
            .withName(ShooterConstants.MotorFollower.MODULE_NAME)
            .withMotorPort(ShooterConstants.MotorFollower.MOTOR_PORT)
            .withCurrentLimit(ShooterConstants.MotorFollower.CURRENT_LIMT)
            .withFollowInverted(ShooterConstants.MotorFollower.FOLLOW_INVERTED);

    shooterMotor =
        new CANSparkFlex(shooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorFollower =
        new CANSparkFlex(
            shooterMotorFollowerConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    shooterMotorEncoder = shooterMotor.getEncoder();
    shooterMotorPidController = shooterMotor.getPIDController();

    MotorConfig.fromMotorConstants(shooterMotor, shooterMotorEncoder, shooterMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(shooterMotorFollower, shooterMotorFollowerConfig)
        .configureMotor()
        .follow(shooterMotor)
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Shooter Velocity", shooterMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Target Velocity", () -> targetVelocity);
    shuffleboardTab.addBoolean("At Target Velocity?", this::atTargetVelocity);
    shuffleboardTab.addDouble("Set point delta", this::debug_getSetpointDelta);
  }

  /**
   * Check if the shooter motor is at the target velocity within a specified tolerance.
   *
   * @return True if the shooter motor is at the target velocity, false otherwise.
   */
  public boolean atTargetVelocity() {
    boolean atTarget =
        Math.abs(targetVelocity - shooterMotorEncoder.getVelocity())
            < ShooterConstants.VELOCITY_TOLERANCE;

    if (atTarget) {
      DEBUG_TIMER.stop();
      debug_setPointDelta = DEBUG_TIMER.get();
    }

    return atTarget;
  }

  public double debug_getSetpointDelta() {
    return debug_setPointDelta;
  }

  /**
   * Set the target velocity for the shooter motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void set(int velocity) {
    DEBUG_TIMER.restart();

    targetVelocity = velocity;
    shooterMotorPidController.setReference(velocity, ControlType.kVelocity);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }
}

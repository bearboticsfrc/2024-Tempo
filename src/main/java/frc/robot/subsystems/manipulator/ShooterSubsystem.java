package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.swerve.MotorConfig;
import frc.bearbotics.swerve.MotorConfig.MotorBuilder;
import frc.bearbotics.swerve.MotorConfig.MotorPIDBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex shooterMotor;
  private CANSparkFlex shooterMotorFollower;

  private SparkPIDController shooterMotorPidController;

  private RelativeEncoder shooterMotorEncoder;

  public ShooterSubsystem() {
    configureMotors();
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void configureMotors() {
    MotorPIDBuilder shooterMotorPid =
        new MotorPIDBuilder()
            .setP(ShooterConstants.Motor.MotorPid.P)
            .setFf(ShooterConstants.Motor.MotorPid.Ff);

    MotorBuilder shooterMotorConfig =
        new MotorBuilder()
            .setModuleName(ShooterConstants.Motor.MODULE_NAME)
            .setMotorPort(ShooterConstants.Motor.MOTOR_PORT)
            .setMotorInverted(ShooterConstants.Motor.INVERTED)
            .setCurrentLimit(ShooterConstants.Motor.CURRENT_LIMT)
            .setMotorPID(shooterMotorPid)
            .setIdleMode(IdleMode.kCoast);

    MotorBuilder shooterMotorFollowerConfig =
        new MotorBuilder()
            .setModuleName(ShooterConstants.MotorFollower.MODULE_NAME)
            .setMotorPort(ShooterConstants.MotorFollower.MOTOR_PORT)
            .setMotorInverted(ShooterConstants.MotorFollower.INVERTED)
            .setCurrentLimit(ShooterConstants.MotorFollower.CURRENT_LIMT);

    shooterMotor =
        new CANSparkFlex(shooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    shooterMotorFollower =
        new CANSparkFlex(
            shooterMotorFollowerConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    shooterMotorEncoder = shooterMotor.getEncoder();
    shooterMotorPidController = shooterMotor.getPIDController();

    MotorConfig.fromMotorConstants(shooterMotor, shooterMotorEncoder, shooterMotorConfig)
        .configureMotor()
        .configurePID(shooterMotorPid)
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(shooterMotorFollower, shooterMotorFollowerConfig)
        .configureMotor()
        .follow(shooterMotor, ShooterConstants.MotorFollower.FOLLOW_INVERTED)
        .burnFlash();
  }

  public boolean atTargetVelocity() {
    return Math.abs(3000 - shooterMotorEncoder.getVelocity()) < 20; // TODO: change
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Shooter Velocity", shooterMotorEncoder::getVelocity);
  }

  public void set(int rpm) {
    shooterMotorPidController.setReference(rpm, ControlType.kVelocity);
  }
}

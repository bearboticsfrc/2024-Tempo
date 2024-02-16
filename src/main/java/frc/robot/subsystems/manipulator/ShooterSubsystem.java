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
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex upperShooterMotor;
  private CANSparkFlex lowerShooterMotor;

  private SparkPIDController upperShooterMotorPidController;
  private SparkPIDController lowerShooterMotorPidController;

  private RelativeEncoder upperShooterMotorEncoder;
  private RelativeEncoder lowerShooterMotorEncoder;

  private int targetVelocity;

  public ShooterSubsystem() {
    configureMotors();
    setupShuffleboardTab(RobotConstants.SHOOTER_SYSTEM_TAB);
  }

  private void configureMotors() {
    MotorPidBuilder upperShooterMotorPidConfig =
        new MotorPidBuilder().withFf(ShooterConstants.UpperMotor.MotorPid.Ff);

    MotorBuilder upperShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.UpperMotor.NAME)
            .withMotorPort(ShooterConstants.UpperMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.UpperMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.UpperMotor.CURRENT_LIMT)
            .withMotorPid(upperShooterMotorPidConfig)
            .withIdleMode(IdleMode.kCoast);

    MotorPidBuilder lowerShooterMotorPid =
        new MotorPidBuilder().withFf(ShooterConstants.LowerMotor.MotorPid.Ff);

    MotorBuilder lowerShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.LowerMotor.NAME)
            .withMotorPort(ShooterConstants.LowerMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.LowerMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.LowerMotor.CURRENT_LIMT)
            .withMotorPid(lowerShooterMotorPid)
            .withIdleMode(IdleMode.kCoast);

    upperShooterMotor =
        new CANSparkFlex(
            upperShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    lowerShooterMotor =
        new CANSparkFlex(
            lowerShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    upperShooterMotorEncoder = upperShooterMotor.getEncoder();
    lowerShooterMotorEncoder = lowerShooterMotor.getEncoder();

    upperShooterMotorPidController = upperShooterMotor.getPIDController();
    lowerShooterMotorPidController = lowerShooterMotor.getPIDController();

    MotorConfig.fromMotorConstants(
            upperShooterMotor, upperShooterMotorEncoder, upperShooterMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(
            lowerShooterMotor, lowerShooterMotorEncoder, lowerShooterMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Upper Shooter Vel", upperShooterMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Lower Shooter Vel", lowerShooterMotorEncoder::getVelocity);

    shuffleboardTab.addDouble("Upper Shooter Cur", upperShooterMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Lower Shooter Cur", lowerShooterMotor::getOutputCurrent);

    shuffleboardTab.addDouble("Upper Shooter Temp", upperShooterMotor::getMotorTemperature);
    shuffleboardTab.addDouble("Lower Shooter Temp", lowerShooterMotor::getMotorTemperature);

    shuffleboardTab.addDouble("Target Velocity", () -> targetVelocity);
    shuffleboardTab.addBoolean("At Target Velocity?", this::atTargetVelocity);
  }

  /**
   * Check if the shooter motor is at the target velocity within a specified tolerance.
   *
   * @return True if the shooter motor is at the target velocity, false otherwise.
   */
  public boolean atTargetVelocity() {
    return Math.abs(targetVelocity - lowerShooterMotorEncoder.getVelocity())
        < ShooterConstants.VELOCITY_TOLERANCE;
  }

  /**
   * Set the target velocity for the shooter motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void set(int velocity) {
    targetVelocity = velocity;

    upperShooterMotorPidController.setReference(velocity, ControlType.kVelocity);
    lowerShooterMotorPidController.setReference(velocity, ControlType.kVelocity);
  }

  /** Stop both shooter motors. */
  public void stop() {
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }
}

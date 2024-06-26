package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkFlex upperShooterMotor;
  private CANSparkFlex lowerShooterMotor;

  private RelativeEncoder upperShooterMotorEncoder;
  private RelativeEncoder lowerShooterMotorEncoder;

  private double targetVelocity;

  public ShooterSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.SHOOTER_SYSTEM_TAB);
    }
  }

  private void configureMotors() {
    MotorPidBuilder upperShooterMotorPidConfig =
        new MotorPidBuilder()
            .withP(ShooterConstants.UpperMotor.MotorPid.P)
            .withFf(ShooterConstants.UpperMotor.MotorPid.Ff);

    MotorPidBuilder lowerShooterMotorPid =
        new MotorPidBuilder()
            .withP(ShooterConstants.LowerMotor.MotorPid.P)
            .withFf(ShooterConstants.LowerMotor.MotorPid.Ff);

    MotorBuilder upperShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.UpperMotor.NAME)
            .withMotorPort(ShooterConstants.UpperMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.UpperMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.UpperMotor.CURRENT_LIMT)
            .withMotorPid(upperShooterMotorPidConfig)
            .withIdleMode(IdleMode.kCoast);

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

    MotorConfig.fromMotorConstants(
            upperShooterMotor, upperShooterMotorEncoder, upperShooterMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder()
        .burnFlash();

    MotorConfig.fromMotorConstants(
            lowerShooterMotor, lowerShooterMotorEncoder, lowerShooterMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder()
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
    return targetVelocity
            - (lowerShooterMotorEncoder.getVelocity() + upperShooterMotorEncoder.getVelocity()) / 2
        < ShooterConstants.VELOCITY_TOLERANCE;
  }

  /**
   * Sets the shooter velocity based on a supplied distance.
   *
   * <p>The actual velocity is determined by a separate method that calculates the optimal velocity
   * based on the given distance.
   *
   * @param distanceSupplier A DoubleSupplier providing the current distance to the target in
   *     meters.
   */
  public void set(DoubleSupplier distanceSupplier) {
    set(getVelocityFromDistance(distanceSupplier.getAsDouble()));
  }

  /**
   * Calculates the shooter velocity based on the distance from the target. This method uses a
   * mathematical model to determine the optimal shooting velocity for a given distance.
   *
   * @param distance The distance from the target in meters.
   * @return The calculated velocity in RPM (rotations per minute) required to shoot the note to the
   *     target at the specified distance.
   */
  private double getVelocityFromDistance(double distance) {
    if (distance <= 2) {
      return 2200;
    } else if (distance >= 5) {
      return 3600;
    }

    return (17868.1 * Math.pow(distance, 0.070036)) - 16423.3;
  }

  /**
   * Set the target velocity for the shooter motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void set(ShooterVelocity velocity) {
    set(velocity.getVelocity());
  }

  /**
   * Set the target velocity for the shooter motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void set(double velocity) {
    targetVelocity = (velocity += ShooterConstants.VELOCITY_COMPENSATION / 2);

    upperShooterMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
    lowerShooterMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  /** Stop both shooter motors. */
  public void stop() {
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }

  public enum ShooterVelocity {
    SUBWOOFER_SHOOT(2000),
    PODIUM_SHOOT(2750),
    BLOOP_SHOOT(950),
    AMP_SHOOT(1200),
    STAGE_SHOOT(3300);

    public double velocity;

    private ShooterVelocity(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}

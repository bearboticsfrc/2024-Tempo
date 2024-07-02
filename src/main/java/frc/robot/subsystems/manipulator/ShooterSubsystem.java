package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex upperShooterMotor;
  private CANSparkFlex lowerShooterMotor;

  private RelativeEncoder upperShooterMotorEncoder;
  private RelativeEncoder lowerShooterMotorEncoder;

  private double targetVelocity;

  public ShooterSubsystem() {
    configureMotors();
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
    OFF(0),
    BLOOP_SHOOT(950);

    public double velocity;

    private ShooterVelocity(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}

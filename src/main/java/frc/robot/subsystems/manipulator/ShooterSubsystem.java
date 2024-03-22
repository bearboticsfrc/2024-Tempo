package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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

  private static final double FUDGE = 500;

  private final String LOGGING_ROOT = "subsystem/shooter/";

  // TOOD: Smelly
  private final DoubleLogEntry upperMotorVelocityLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "upper_motor/velocity");
  private final DoubleLogEntry lowerMotorVelocityLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "lower_motor/velocity");
  private final DoubleLogEntry upperMotorCurrentLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "upper_motor/current");
  private final DoubleLogEntry lowerMotorCurrentLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "lower_motor/current");
  private final DoubleLogEntry upperMotorTemperatureLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "upper_motor/temperature");
  private final DoubleLogEntry lowerMotorTemperatureLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "lower_motor/temperature");
  private final DoubleLogEntry targetVelocityLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "target_velocity");

  private final BooleanLogEntry setpointLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "at_setpoint");

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

  @Override
  public void periodic() {
    updateDataLogs();
  }

  public void updateDataLogs() {
    upperMotorVelocityLogEntry.append(upperShooterMotorEncoder.getVelocity());
    lowerMotorVelocityLogEntry.append(lowerShooterMotorEncoder.getVelocity());

    upperMotorCurrentLogEntry.append(upperShooterMotor.getOutputCurrent());
    lowerMotorCurrentLogEntry.append(lowerShooterMotor.getOutputCurrent());

    upperMotorTemperatureLogEntry.append(upperShooterMotor.getMotorTemperature());
    lowerMotorTemperatureLogEntry.append(lowerShooterMotor.getMotorTemperature());

    targetVelocityLogEntry.append(targetVelocity);

    setpointLogEntry.append(atTargetVelocity());
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

  public void set(DoubleSupplier distanceSupplier) {
    set(getVelocityFromDistance(distanceSupplier.getAsDouble()));
  }

  private double getVelocityFromDistance(double distance) {
    if (distance <= 2) {
      return 2200 + FUDGE;
    } else if (distance >= 5) {
      return 3600 + FUDGE;
    }

    return (17868.1 * Math.pow(distance, 0.070036)) - 16423.3 + FUDGE;
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
    upperShooterMotor.getPIDController().setReference(0, ControlType.kVelocity);
    lowerShooterMotor.getPIDController().setReference(0, ControlType.kVelocity);
  }

  public enum ShooterVelocity {
    SUBWOOFER_SHOOT(2000 + FUDGE),
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

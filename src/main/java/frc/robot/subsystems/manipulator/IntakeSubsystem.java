package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private final String LOGGING_ROOT = "subsystem/intake/";

  // TODO: Smelly
  private final DoubleLogEntry rollerVelocityLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "roller/velocity");
  private final DoubleLogEntry feederVelocityLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "feeder/velocity");

  private final DoubleLogEntry rollerCurrentLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "roller/current");
  private final DoubleLogEntry feederCurrentLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "feeder/current");

  private final DoubleLogEntry rollerTemperatureLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "roller/temperature");
  private final DoubleLogEntry feederTemperatureLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "feeder/temperature");

  private final BooleanLogEntry bottomBeambreakLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "beambreak/bottom");
  private final BooleanLogEntry topBeambreakLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "beambreak/top");
  private final BooleanLogEntry leftBeambreakLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "beambreak/left");
  private final BooleanLogEntry rightBeambreakLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "beambreak/right");
  private final BooleanLogEntry rollerBeambreakLogEntry =
      new BooleanLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "beambreak/roller");

  private CANSparkMax rollerMotor;
  private CANSparkMax feederMotor;

  private RelativeEncoder rollerMotorEncoder;
  private RelativeEncoder feederMotorEncoder;

  private final DigitalInput topBeamBreak =
      new DigitalInput(IntakeConstants.TOP_BEAM_BREAK_CHANNEL);
  private final DigitalInput leftBeamBreak =
      new DigitalInput(IntakeConstants.LEFT_BEAM_BREAK_CHANNEL);
  private final DigitalInput rightBeamBreak =
      new DigitalInput(IntakeConstants.RIGHT_BEAM_BREAK_CHANNEL);
  private final DigitalInput bottomBeamBreak =
      new DigitalInput(IntakeConstants.BOTTOM_BEAM_BREAK_CHANNEL);
  private final DigitalInput rollerBeamBreak =
      new DigitalInput(IntakeConstants.ROLLER_BEAM_BREAK_CHANNEL);

  public IntakeSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.INTAKE_SYSTEM_TAB);
    }
  }

  @Override
  public void periodic() {
    updateDataLogs();
  }

  /** Update data logs. */
  private void updateDataLogs() {
    rollerVelocityLogEntry.append(rollerMotorEncoder.getVelocity());
    feederVelocityLogEntry.append(feederMotorEncoder.getVelocity());

    rollerCurrentLogEntry.append(rollerMotor.getOutputCurrent());
    feederCurrentLogEntry.append(feederMotor.getOutputCurrent());

    rollerTemperatureLogEntry.append(rollerMotor.getMotorTemperature());
    feederTemperatureLogEntry.append(feederMotor.getMotorTemperature());

    topBeambreakLogEntry.append(topBeamBreak.get());
    bottomBeambreakLogEntry.append(bottomBeamBreak.get());
    rightBeambreakLogEntry.append(rightBeamBreak.get());
    leftBeambreakLogEntry.append(leftBeamBreak.get());
    rollerBeambreakLogEntry.append(rollerBeamBreak.get());
  }

  private void configureMotors() {
    MotorBuilder rollerMotorBuilder =
        new MotorBuilder()
            .withName(IntakeConstants.RollerMotor.NAME)
            .withMotorPort(IntakeConstants.RollerMotor.MOTOR_PORT)
            .withMotorInverted(IntakeConstants.RollerMotor.INVERTED)
            .withCurrentLimit(IntakeConstants.RollerMotor.CURRENT_LIMT)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder feederMotorBuilder =
        new MotorBuilder()
            .withName(IntakeConstants.FeederMotor.NAME)
            .withMotorPort(IntakeConstants.FeederMotor.MOTOR_PORT)
            .withMotorInverted(IntakeConstants.FeederMotor.INVERTED)
            .withCurrentLimit(IntakeConstants.FeederMotor.CURRENT_LIMT);

    rollerMotor =
        new CANSparkMax(rollerMotorBuilder.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    feederMotor =
        new CANSparkMax(feederMotorBuilder.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    rollerMotorEncoder = rollerMotor.getEncoder();
    feederMotorEncoder = feederMotor.getEncoder();

    MotorConfig.fromMotorConstants(rollerMotor, rollerMotorEncoder, rollerMotorBuilder)
        .configureMotor()
        .configureEncoder()
        .burnFlash();

    MotorConfig.fromMotorConstants(feederMotor, feederMotorEncoder, feederMotorBuilder)
        .configureMotor()
        .configureEncoder()
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Roller Velocity", rollerMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Feeder Velocity", feederMotorEncoder::getVelocity);
    shuffleboardTab.addBoolean("Bottom Beam Break", bottomBeamBreak::get);
    shuffleboardTab.addBoolean("Left Beam Break", leftBeamBreak::get);
    shuffleboardTab.addBoolean("Right Beam Break", rightBeamBreak::get);
    shuffleboardTab.addBoolean("Top Beam Break", topBeamBreak::get);
    shuffleboardTab.addBoolean("Roller Beam Break", rollerBeamBreak::get);
  }

  public boolean isNoteInTop() {
    return !topBeamBreak.get();
  }

  /**
   * Return a boolean whether a note has triggered the roller beam break sensor.
   *
   * @return True if a note is not in the roller, false otherwise.
   */
  public boolean isNoteInRoller() {
    return !rollerBeamBreak.get();
  }

  /**
   * Return a boolean whether a note has triggered the top and either of the two side beam break
   * sensors.
   *
   * @return True if a note is not in the feeder, false otherwise.
   */
  public boolean isNoteInFeeder() {
    return !topBeamBreak.get() && isNoteInSide();
  }

  /**
   * Return a boolean whether a note has triggered either of the two side beam break sensors.
   *
   * @return True if a note is not in the feeder, false otherwise.
   */
  public boolean isNoteInSide() {
    return !leftBeamBreak.get() || !rightBeamBreak.get();
  }

  /**
   * Set the roller motor to the specified intake speed.
   *
   * @param speed The desired intake speed for the roller.
   */
  public void setRoller(IntakeSpeed speed) {
    rollerMotor.set(speed.getSpeed());
  }

  /**
   * Set the feeder motor to the specified intake speed.
   *
   * @param speed The desired intake speed for the feeder.
   */
  public void setFeeder(IntakeSpeed speed) {
    feederMotor.set(speed.getSpeed());
  }

  /** Enum representing different intake speeds. */
  public enum IntakeSpeed {
    REVERSE(-1),
    OFF(0),
    TENTH(0.1),
    QUARTER(0.25),
    HALF(0.5),
    FULL(1);

    private final double speed;

    /**
     * Constructor for IntakeSpeed.
     *
     * @param speed The speed value associated with the intake speed.
     */
    private IntakeSpeed(double speed) {
      this.speed = speed;
    }

    /**
     * Get the speed value associated with the intake speed.
     *
     * @return The speed value.
     */
    public double getSpeed() {
      return speed;
    }
  }
}

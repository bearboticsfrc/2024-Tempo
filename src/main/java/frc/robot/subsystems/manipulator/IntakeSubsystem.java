package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
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
    setupShuffleboardTab(DriveConstants.MANIPULATOR_SYSTEM_TAB);
  }

  private void configureMotors() {
    MotorBuilder rollerMotorBuilder =
        new MotorBuilder()
            .withModuleName(IntakeConstants.RollerMotor.MODULE_NAME)
            .withMotorPort(IntakeConstants.RollerMotor.MOTOR_PORT)
            .withMotorInverted(IntakeConstants.RollerMotor.INVERTED)
            .withCurrentLimit(IntakeConstants.RollerMotor.CURRENT_LIMT)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder feederMotorBuilder =
        new MotorBuilder()
            .withModuleName(IntakeConstants.FeederMotor.MODULE_NAME)
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
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();

    MotorConfig.fromMotorConstants(feederMotor, feederMotorEncoder, feederMotorBuilder)
        .configureMotor()
        .configureEncoder(Rotation2d.fromRotations(0))
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

  /**
   * Return a boolean whether a note has triggered the roller beam break sensor.
   *
   * @return True if a note is not in the roller, false otherwise.
   */
  public boolean isNoteInRoller() {
    return !rollerBeamBreak.get();
  }

  /**
   * Return a boolean whether a note has triggered the feeder beam break sensor.
   *
   * @return True if a note is not in the feeder, false otherwise.
   */
  public boolean isNoteInFeeder() {
    return !topBeamBreak.get();
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
    OFF(0),
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

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.swerve.MotorConfig;
import frc.bearbotics.swerve.MotorConfig.MotorBuilder;
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
            .setModuleName(IntakeConstants.RollerMotor.MODULE_NAME)
            .setMotorPort(IntakeConstants.RollerMotor.MOTOR_PORT)
            .setMotorInverted(IntakeConstants.RollerMotor.INVERTED)
            .setCurrentLimit(IntakeConstants.RollerMotor.CURRENT_LIMT)
            .setIdleMode(IdleMode.kCoast);

    MotorBuilder feederMotorBuilder =
        new MotorBuilder()
            .setModuleName(IntakeConstants.FeederMotor.MODULE_NAME)
            .setMotorPort(IntakeConstants.FeederMotor.MOTOR_PORT)
            .setMotorInverted(IntakeConstants.FeederMotor.INVERTED)
            .setCurrentLimit(IntakeConstants.FeederMotor.CURRENT_LIMT);

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

  public boolean isNoteInRoller() {
    return !rollerBeamBreak.get();
  }

  public boolean isNoteInFeeder() {
    return !topBeamBreak.get();
  }

  public void setRoller(RollerIntakeSpeed speed) {
    rollerMotor.set(speed.getSpeed());
  }

  public void setFeeder(FeederIntakeSpeed speed) {
    feederMotor.set(speed.getSpeed());
  }

  public enum RollerIntakeSpeed {
    OFF(0),
    HALF(1);

    private final double speed;

    private RollerIntakeSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return speed;
    }
  }

  public enum FeederIntakeSpeed {
    OFF(0),
    HALF(0.5),
    FULL(1);

    private final double speed;

    private FeederIntakeSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return speed;
    }
  }
}

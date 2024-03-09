package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import java.util.function.DoubleSupplier;

public class ManipulatorSubsystem extends SubsystemBase {
  public final IntakeSubsystem intakeSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final ClimberSubsystem climberSubsystem;
  public final ArmSubsystem armSubsystem;

  /**
   * Constructor for the ManipulatorSubsystem class. Initializes intake, shooter, climber, and arm
   * subsystems.
   */
  public ManipulatorSubsystem() {
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climberSubsystem = new ClimberSubsystem();
    armSubsystem = new ArmSubsystem();
  }

  /**
   * Check if a note is in the roller.
   *
   * @return True if a note is in the roller, false otherwise.
   */
  public boolean isNoteInRoller() {
    return intakeSubsystem.isNoteInRoller();
  }

  /**
   * Check if a note is in the feeder.
   *
   * @return True if a note is in the feeder, false otherwise.
   */
  public boolean isNoteInFeeder() {
    return intakeSubsystem.isNoteInFeeder();
  }

  /**
   * Generates the generic intake command.
   *
   * @return The generated intake command.
   */
  public Command getIntakeCommand() {
    return Commands.either(
        Commands.none(),
        Commands.sequence(
            Commands.parallel(
                getRollerRunCommand(IntakeSpeed.FULL), getFeederRunCommand(IntakeSpeed.QUARTER)),
            Commands.waitUntil(intakeSubsystem::isNoteInSide),
            getFeederRunCommand(IntakeSpeed.TENTH),
            Commands.waitUntil(intakeSubsystem::isNoteInFeeder),
            getIntakeStopCommand()),
        intakeSubsystem::isNoteInFeeder);
  }

  /**
   * Get a command to run the climber to a specified speed.
   *
   * @param speedSupplier The desired climber speed supplier.
   * @return The RunCommand to set the climber speed.
   */
  public Command getClimberRunCommand(DoubleSupplier speedSupplier) {
    return Commands.run(() -> climberSubsystem.set(speedSupplier.getAsDouble()), this);
  }

  /**
   * Get a command to home the climber.
   *
   * @return The Command to home the climber.
   */
  public Command getClimberHomeCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> climberSubsystem.set(-0.5), this),
            Commands.waitUntil(climberSubsystem::isClimberHome),
            Commands.runOnce(() -> climberSubsystem.stop(), this))
        .withName("Home Climber");
  }

  /**
   * Get a command to run the roller at a specified speed.
   *
   * @param speed The desired roller intake speed.
   * @return The Command to set the roller speed.
   */
  public Command getRollerRunCommand(IntakeSpeed speed) {
    return Commands.runOnce(() -> intakeSubsystem.setRoller(speed));
  }

  /**
   * Get a command to run the feeder at a specified speed.
   *
   * @param speed The desired feeder intake speed.
   * @return The Command to set the feeder speed.
   */
  public Command getFeederRunCommand(IntakeSpeed speed) {
    return Commands.runOnce(() -> intakeSubsystem.setFeeder(speed));
  }

  /**
   * Get a command to stop both the roller and feeder.
   *
   * @return The command to stop the roller and feeder.
   */
  public Command getIntakeStopCommand() {
    return Commands.parallel(
        getRollerRunCommand(IntakeSpeed.OFF), getFeederRunCommand(IntakeSpeed.OFF));
  }

  /**
   * Get a command to feed the intake by starting the feeder, waiting until a note is in the feeder,
   * and then stopping both roller and feeder.
   *
   * @return The Command for feeding the intake.
   */
  public Command getIntakeFeedCommand() {
    return Commands.sequence(
        getFeederRunCommand(IntakeSpeed.FULL),
        Commands.waitUntil(() -> !intakeSubsystem.isNoteInFeeder()),
        getIntakeStopCommand());
  }

  /**
   * Get a command to run the shooter at a specified velocity.
   *
   * @param velocity The desired shooter velocity.
   * @return The Command to set the shooter velocity.
   */
  public Command getShooterRunCommand(ShooterVelocity velocity) {
    return Commands.runOnce(() -> shooterSubsystem.set(velocity));
  }

  /**
   * Get a command to run the shooter at a calcluated velocity using the distance supplier.
   *
   * @param distanceSupplier The supplier for the distance.
   * @return The Command to set the shooter velocity.
   */
  public Command getShooterRunCommand(DoubleSupplier distanceSupplier) {
    return Commands.runOnce(() -> shooterSubsystem.set(distanceSupplier));
  }

  /**
   * Get a command to stop the shooter.
   *
   * @return The Command to stop the shooter.
   */
  public Command getShootStopCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> shooterSubsystem.stop()), getArmRunCommand(ArmPosition.HOME));
  }

  /**
   * Get a command to execute the entire shooting sequence, including feeding notes.
   *
   * @return The Command for shooting.
   */
  public Command getShootCommand() {
    return Commands.sequence(getIntakeFeedCommand(), getShootStopCommand());
  }

  /**
   * Get a command to run the arm to a specified position.
   *
   * @param position The desired arm position.
   * @return The Command to set the arm position.
   */
  public Command getArmRunCommand(ArmPosition position) {
    return Commands.runOnce(() -> armSubsystem.set(position));
  }

  /**
   * Get a command to run the arm at a calcluated position using the distance supplier.
   *
   * @param distanceSupplier The supplier for the distance.
   * @return The Command to set the arm position.
   */
  public Command getArmRunCommand(DoubleSupplier distanceSupplier) {
    return Commands.runOnce(() -> armSubsystem.set(distanceSupplier));
  }

  /**
   * Get a command to stop the arm.
   *
   * @return The Command to stop the arm.
   */
  public Command getArmStopCommand() {
    return Commands.runOnce(() -> armSubsystem.stop(), this);
  }

  /**
   * Get a command to execute the entire shooting sequence for the podium.
   *
   * @return The command.
   */
  public Command getPodiumShootCommand() {
    return getShootCommand(ArmPosition.PODIUM_SHOOT, ShooterVelocity.PODIUM_SHOOT);
  }

  /**
   * Get a command to execute the entire shooting sequence for the stage.
   *
   * @return The command.
   */
  public Command getStageShootCommand() {
    return getShootCommand(ArmPosition.STAGE_SHOOT, ShooterVelocity.STAGE_SHOOT);
  }

  /**
   * Get a command to execute the entire shooting sequence for the amp.
   *
   * @return The command.
   */
  public Command getAmpShootCommand() {
    return getShootCommand(ArmPosition.AMP_SHOOT, ShooterVelocity.AMP_SHOOT);
  }

  /**
   * Get a command to execute the entire shooting sequence for the subwoofer setup.
   *
   * @return The command.
   */
  public Command getSubwooferShootCommand() {
    return getShootCommand(ArmPosition.HOME, ShooterVelocity.SUBWOOFER_SHOOT);
  }

  private Command getShootCommand(ArmPosition armPosition, ShooterVelocity shooterVelocity) {
    return Commands.sequence(
        Commands.parallel(
            getArmPrepareCommand(armPosition), getShooterPrepareCommand(shooterVelocity)),
        getShootCommand());
  }

  /**
   * Get a command to execute the entire shooting sequence for a specified distance.
   *
   * @param distanceSupplier The supplier for the shooting distance.
   * @return The command.
   */
  public Command getAutoShootCommand(DoubleSupplier distanceSupplier) {
    return Commands.sequence(
        Commands.parallel(
            getArmPrepareCommand(distanceSupplier), getShooterPrepareCommand(distanceSupplier)),
        getShootCommand());
  }

  /**
   * Get a command to prepare the shooter and arm for a specified distance using a supplier.
   *
   * @param distanceSupplier The supplier for the shooting distance.
   * @return The command.
   */
  public Command getShooterPrepareCommad(DoubleSupplier distanceSupplier) {
    return Commands.either(
        Commands.parallel(
            getArmPrepareCommand(distanceSupplier), getShooterPrepareCommand(distanceSupplier)),
        Commands.none(),
        this::isNoteInFeeder);
  }

  /**
   * Get a command to prepare the shooter to run at a specified velocity, and wait until the shooter
   * reaches the target velocity.
   *
   * @param velocity The desired shooter velocity.
   * @return The command.
   */
  private Command getShooterPrepareCommand(ShooterVelocity velocity) {
    return Commands.sequence(
        getShooterRunCommand(velocity), Commands.waitUntil(shooterSubsystem::atTargetVelocity));
  }

  /**
   * Get a command to run the shooter at a calcluated velocity using the distance supplier, and wait
   * until the shooter reaches target velocity.
   *
   * @param distanceSupplier The supplier for the shooter velocity.
   * @return The command.
   */
  private Command getShooterPrepareCommand(DoubleSupplier distanceSupplier) {
    return Commands.sequence(
        getShooterRunCommand(distanceSupplier),
        Commands.waitUntil(shooterSubsystem::atTargetVelocity));
  }

  /**
   * Get a command to run the arm to a specified position, and wait until the arm reaches target
   * position.
   *
   * @param position The desired arm position.
   * @return The command.
   */
  private Command getArmPrepareCommand(ArmPosition position) {
    return Commands.sequence(
        getArmRunCommand(position), Commands.waitUntil(armSubsystem::atTargetSetpoint));
  }

  /**
   * Get a command to run the arm at a calcluated position using the distance supplier, and wait
   * until the arm reaches target position.
   *
   * @param distanceSupplier The supplier for the arm position.
   * @return The command.
   */
  private Command getArmPrepareCommand(DoubleSupplier distanceSupplier) {
    return Commands.sequence(
        getArmRunCommand(distanceSupplier), Commands.waitUntil(armSubsystem::atTargetSetpoint));
  }
}

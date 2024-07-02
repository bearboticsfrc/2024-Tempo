package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  /**
   * Constructor for the ManipulatorSubsystem class. Initializes intake, shooter, climber, and arm
   * subsystems.
   */
  public ManipulatorSubsystem() {
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
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
   * Check if a note is in either the roller or feeder.
   *
   * @return True if a note is in the roller or feeder, false otherwise.
   */
  public boolean isNoteInIntake() {
    return isNoteInRoller() || isNoteInFeeder();
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
  public Command getShooterStopCommand() {
    return Commands.parallel(
        getShooterRunCommand(ShooterVelocity.OFF), getFeederRunCommand(IntakeSpeed.OFF));
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
        getShooterStopCommand());
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
   * Get a command to prepare the shooter to run at a specified velocity, and wait until the shooter
   * reaches the target velocity.
   *
   * @param velocity The desired shooter velocity.
   * @return The command.
   */
  public Command getShooterPrepareCommand(ShooterVelocity velocity) {
    return Commands.sequence(
        getShooterRunCommand(velocity), Commands.waitUntil(shooterSubsystem::atTargetVelocity));
  }

  /**
   * Get a command to execute the entire shooting sequence for a bloop shot. A bloop shoot is
   * designed to gracefully eject a note from the shooter. Does not bloopy bloop.
   *
   * @return The command.
   */
  public Command getBloopShootCommand() {
    return Commands.sequence(
        getShooterPrepareCommand(ShooterVelocity.BLOOP_SHOOT), getIntakeFeedCommand());
  }
}

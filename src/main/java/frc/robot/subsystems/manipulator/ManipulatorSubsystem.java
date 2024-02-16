package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import java.util.function.DoubleSupplier;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final ArmSubsystem armSubsystem;

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
   * Generates the generic intake command with the following sequence:
   *
   * <p>1. Start roller and feeder motors.
   *
   * <p>2. Wait for the top beam break to be tripped.
   *
   * <p>3. Stop roller and feeder motors.
   *
   * @return The generated intake command.
   */
  public SequentialCommandGroup getIntakeCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getRollerRunCommand(IntakeSpeed.FULL), getFeederRunCommand(IntakeSpeed.FEED)),
        new WaitUntilCommand(intakeSubsystem::isNoteInFeeder),
        getIntakeStopCommand());
  }

  /**
   * Get a command to run the roller at a specified speed.
   *
   * @param speed The desired roller intake speed.
   * @return The InstantCommand to set the roller speed.
   */
  public InstantCommand getRollerRunCommand(IntakeSpeed speed) {
    return new InstantCommand(() -> intakeSubsystem.setRoller(speed));
  }

  /**
   * Get a command to run the feeder at a specified speed.
   *
   * @param speed The desired feeder intake speed.
   * @return The InstantCommand to set the feeder speed.
   */
  public InstantCommand getFeederRunCommand(IntakeSpeed speed) {
    return new InstantCommand(() -> intakeSubsystem.setFeeder(speed));
  }

  /**
   * Get a command to stop both the roller and feeder.
   *
   * @return The command to stop the roller and feeder.
   */
  public ParallelCommandGroup getIntakeStopCommand() {
    return new ParallelCommandGroup(
        getRollerRunCommand(IntakeSpeed.OFF), getFeederRunCommand(IntakeSpeed.OFF));
  }

  /**
   * Get a command to feed the intake:
   *
   * <p>1. Start the feeder at full speed.
   *
   * <p>2. Wait until the beam break is no longer tripped.
   *
   * <p>3. Stop both the roller and feeder.
   *
   * @return The SequentialCommandGroup for feeding the intake.
   */
  public SequentialCommandGroup getIntakeFeedCommand() {
    return new SequentialCommandGroup(
        getFeederRunCommand(IntakeSpeed.FULL),
        new WaitUntilCommand(() -> !intakeSubsystem.isNoteInFeeder()),
        new WaitCommand(0.02), // Waits until feeder motors are not holding a note anymore.
        getIntakeStopCommand());
  }

  /**
   * Get a command to shoot the note at a specified velocity.
   *
   * @param velocity The desired velocity for shooting.
   * @return The SequentialCommandGroup to shoot the note.
   */
  public SequentialCommandGroup getShootCommand(int velocity) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> shooterSubsystem.set(velocity)),
        new WaitUntilCommand(shooterSubsystem::atTargetVelocity),
        getIntakeFeedCommand());
  }

  /**
   * Get a command to stop the shooter.
   *
   * @return The InstantCommand to stop the shooter.
   */
  public InstantCommand getShootStopCommand() {
    return new InstantCommand(() -> shooterSubsystem.stop());
  }

  /**
   * Get a command to run the climber to a specified speed.
   *
   * @param speedSupplier The desired climber speed supplier.
   * @return The RunCommand to set the climber speed.
   */
  public RunCommand getClimberRunCommand(DoubleSupplier speedSupplier) {
    return new RunCommand(() -> climberSubsystem.set(speedSupplier.getAsDouble()), this);
  }

  /**
   * Get a command to home the climber.
   *
   * @return The SequentialCommandGroup to home the climber.
   */
  public SequentialCommandGroup getClimberHomeCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> climberSubsystem.set(-0.5), this),
        new WaitUntilCommand(climberSubsystem::isClimberHome),
        new InstantCommand(() -> climberSubsystem.stop()));
  }

  public InstantCommand getArmRunCommand(ArmPosition position) {
    return new InstantCommand(() -> armSubsystem.set(position));
  }

  public InstantCommand getArmStopCommand() {
    return new InstantCommand(() -> armSubsystem.stop());
  }
}

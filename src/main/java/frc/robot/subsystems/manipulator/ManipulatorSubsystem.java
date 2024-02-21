package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
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
   * Check if a note is in the feeder.
   *
   * @return True if a note is in the feeder, false otherwise.
   */
  public boolean isNoteInFeeder() {
    return intakeSubsystem.isNoteInFeeder();
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

  /**
   * Generates the generic intake command with the following sequence:
   *
   * <p>1. Start roller and feeder motors.
   *
   * <p>2. Wait for the side beam breaks to be tripped.
   *
   * <p>3. Run feeder motor at slower speed.
   *
   * <p>4. Stop roller and feeder motors.
   *
   * @return The generated intake command.
   */
  public ConditionalCommand getIntakeCommand() {
    return new ConditionalCommand(
        new InstantCommand(),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                getRollerRunCommand(IntakeSpeed.FULL), getFeederRunCommand(IntakeSpeed.QUARTER)),
            new WaitUntilCommand(intakeSubsystem::isNoteInSide),
            getFeederRunCommand(IntakeSpeed.TENTH),
            new WaitUntilCommand(intakeSubsystem::isNoteInFeeder),
            getIntakeStopCommand()),
        intakeSubsystem::isNoteInFeeder);
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

  public SequentialCommandGroup getSpecialIntakeCommand() {
    return new SequentialCommandGroup(
        getIntakeCommand(),
        new ConditionalCommand(
            new InstantCommand(),
            getIntakeStopCommand(),
            () -> isNoteInRoller() || isNoteInFeeder()));
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
        getIntakeStopCommand());
  }

  public InstantCommand getShooterRunCommand(ShooterVelocity velocity) {
    return new InstantCommand(() -> shooterSubsystem.set(velocity));
  }

  public InstantCommand getShooterRunCommand(DoubleSupplier doubleSupplier) {
    return new InstantCommand(() -> shooterSubsystem.set(doubleSupplier));
  }

  /**
   * Get a command to stop the shooter.
   *
   * @return The InstantCommand to stop the shooter.
   */
  public ParallelCommandGroup getShootStopCommand() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> shooterSubsystem.stop()), getArmRunCommand(ArmPosition.HOME));
  }

  private SequentialCommandGroup getShootCommand() {
    return new SequentialCommandGroup(getIntakeFeedCommand(), getShootStopCommand());
  }

  /**
   * 1. Run arm and wait until at setpoint and run shooter and wait until at velocity.
   *
   * <p>2. Feed note.
   *
   * @return The command.
   */
  public SequentialCommandGroup getPodiumShootCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getArmPrepareCommand(ArmPosition.PODIUM_SHOOT),
            getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)),
        getShootCommand());
  }

  public SequentialCommandGroup getWingShootCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getArmPrepareCommand(ArmPosition.WING_SHOOT),
            getShooterPrepareCommand(ShooterVelocity.WING_SHOOT)),
        getShootCommand());
  }

  public SequentialCommandGroup getAmpShootCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getArmPrepareCommand(ArmPosition.AMP_SHOOT),
            getShooterPrepareCommand(ShooterVelocity.AMP_SHOOT)),
        getShootCommand());
  }

  public SequentialCommandGroup getAutoShootCommand(DoubleSupplier distanceSupplier) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getArmPrepareCommand(distanceSupplier), getShooterPrepareCommand(distanceSupplier)),
        getShootCommand());
  }

  public InstantCommand getArmRunCommand(ArmPosition position) {
    return new InstantCommand(() -> armSubsystem.set(position));
  }

  public InstantCommand getArmRunCommand(DoubleSupplier distanceSupplier) {
    return new InstantCommand(() -> armSubsystem.set(distanceSupplier));
  }

  private SequentialCommandGroup getShooterPrepareCommand(ShooterVelocity velocity) {
    return new SequentialCommandGroup(
        getShooterRunCommand(velocity), new WaitUntilCommand(shooterSubsystem::atTargetVelocity));
  }

  private SequentialCommandGroup getShooterPrepareCommand(DoubleSupplier distanceSupplier) {
    return new SequentialCommandGroup(
        getShooterRunCommand(distanceSupplier),
        new WaitUntilCommand(shooterSubsystem::atTargetVelocity));
  }

  private SequentialCommandGroup getArmPrepareCommand(ArmPosition position) {
    return new SequentialCommandGroup(
        getArmRunCommand(position), new WaitUntilCommand(armSubsystem::atTargetSetpoint));
  }

  private SequentialCommandGroup getArmPrepareCommand(DoubleSupplier distanceSupplier) {
    return new SequentialCommandGroup(
        getArmRunCommand(distanceSupplier), new WaitUntilCommand(armSubsystem::atTargetSetpoint));
  }

  public InstantCommand getArmStopCommand() {
    return new InstantCommand(() -> armSubsystem.stop());
  }

  /**
   * 1. Run arm and wait until at setpoint and run shooter and wait until at velocity.
   *
   * <p>2. Feed note.
   *
   * @return The command.
   */
  public SequentialCommandGroup getSubwooferShootCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(getShooterPrepareCommand(ShooterVelocity.SUBWOOFER_SHOOT)),
        getShootCommand());
  }
}

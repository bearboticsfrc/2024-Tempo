package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.manipulator.ClimberSubsystem.ClimberPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.FeederIntakeSpeed;
import frc.robot.subsystems.manipulator.IntakeSubsystem.RollerIntakeSpeed;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ClimberSubsystem climberSubsystem;

  public ManipulatorSubsystem() {
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climberSubsystem = new ClimberSubsystem();
  }

  private void setupShuffleboardTab() {}

  @Override
  public void periodic() {}

  public boolean isNoteInRoller() {
    return intakeSubsystem.isNoteInRoller();
  }

  public Command getShootCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> shooterSubsystem.set(3000)),
        new WaitUntilCommand(shooterSubsystem::atTargetVelocity),
        getIntakeFeedCommand());
  }

  public Command getShootStopCommand() {
    return new InstantCommand(() -> shooterSubsystem.set(0));
  }

  public Command getClimberRunCommand(ClimberPosition position) {
    return new InstantCommand(() -> climberSubsystem.set(position));
  }

  public Command getClimberHomeCommand() {
    return new InstantCommand(() -> climberSubsystem.set(0.25));
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
   * <p>
   *
   * @return The generated intake command.
   */
  public SequentialCommandGroup getIntakeRunCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            getRollerRunCommand(RollerIntakeSpeed.HALF),
            getFeederRunCommand(FeederIntakeSpeed.HALF)),
        new WaitUntilCommand(intakeSubsystem::isNoteInFeeder),
        getIntakeStopCommand());
  }

  public InstantCommand getRollerRunCommand(RollerIntakeSpeed speed) {
    return new InstantCommand(() -> intakeSubsystem.setRoller(speed));
  }

  public InstantCommand getFeederRunCommand(FeederIntakeSpeed speed) {
    return new InstantCommand(() -> intakeSubsystem.setFeeder(speed));
  }

  public ParallelCommandGroup getIntakeStopCommand() {
    return new ParallelCommandGroup(
        getRollerRunCommand(RollerIntakeSpeed.OFF), getFeederRunCommand(FeederIntakeSpeed.OFF));
  }

  public SequentialCommandGroup getIntakeFeedCommand() {
    return new SequentialCommandGroup(
        getFeederRunCommand(FeederIntakeSpeed.FULL),
        new WaitUntilCommand(() -> !intakeSubsystem.isNoteInFeeder()),
        getIntakeStopCommand());
  }
  /**
   * Generates the fancy intake command with the following sequence:
   *
   * <p>1. Run intake motor.
   *
   * <p>2. Wait until bottom beam break is tripped.
   *
   * <p>3. Run feeder motor.
   *
   * <p>4. Wait until left and right beam break is tripped.
   *
   * <p>5. Stop roller motor.
   *
   * <p>6. Wait until top beam break is tripped.
   *
   * <p>7. Stop feeder motor.
   *
   * <p>
   *
   * @return The generated intake command.
   *     <p>public SequentialCommandGroup getFancyIntakeCommand() { return new
   *     SequentialCommandGroup( getRollerRunCommand(RollerIntakeSpeed.HALF), new
   *     WaitUntilCommand(() -> !bottomBeamBreak.get()),
   *     getFeederRunCommand(FeederIntakeSpeed.FULL), new WaitUntilCommand(() ->
   *     !leftBeamBreak.get() && !rightBeamBreak.get()), getRollerRunCommand(RollerIntakeSpeed.OFF),
   *     new WaitUntilCommand(() -> !topBeamBreak.get()),
   *     getFeederRunCommand(FeederIntakeSpeed.OFF)); }
   */
}

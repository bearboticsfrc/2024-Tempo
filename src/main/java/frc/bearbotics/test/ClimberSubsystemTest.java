package frc.bearbotics.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.manipulator.ClimberConstants;
import frc.robot.subsystems.manipulator.ClimberSubsystem;
import java.util.List;
import java.util.Map;

public class ClimberSubsystemTest extends AbstractTestCommand {
  private final String NAME = "Climber Subsystem Test";

  private final ClimberSubsystem climberSubsystem;

  private Map<String, Boolean> outcomes = Map.of("Climber Home", false);

  public ClimberSubsystemTest(ClimberSubsystem climberSubsystem, ShuffleboardTab shuffleboardTab) {
    this.climberSubsystem = climberSubsystem;

    setupShuffleboardTab(shuffleboardTab);
  }

  @Override
  protected double getTestTimeout() {
    return ClimberConstants.Test.TIMEOUT;
  }

  @Override
  protected void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    int row = 2, column = 6;

    for (String module : outcomes.keySet()) {
      shuffleboardTab.addBoolean(module, () -> outcomes.get(module)).withPosition(column, row);
      column = (column + 1) % 4;
      row = (column == 0) ? 1 : row;
    }
  }

  @Override
  protected Map<String, Boolean> getOutcomes() {
    return outcomes;
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.set(0);

    List<String> failed = getFailedTests();

    if (failed.size() > 0) {
      DriverStation.reportError(
          NAME + failed.size() + " motor failed test(s): " + String.join(", ", failed), false);
    }
  }

  @Override
  protected Command getTestCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> climberSubsystem.set(0.5)),
        Commands.waitSeconds(ClimberConstants.Test.WAIT),
        Commands.sequence(
            Commands.runOnce(() -> climberSubsystem.set(-0.5)),
            Commands.waitUntil(climberSubsystem::isClimberHome),
            Commands.runOnce(() -> climberSubsystem.stop())),
        Commands.runOnce(() -> setPass("Climber Home", climberSubsystem.isClimberHome())));
  }
}

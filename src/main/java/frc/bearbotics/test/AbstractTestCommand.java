package frc.bearbotics.test;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveModuleConstants;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public abstract class AbstractTestCommand extends Command {
  protected abstract Command getTestCommand();

  protected abstract Map<String, Boolean> getOutcomes();

  protected abstract void setupShuffleboardTab(ShuffleboardTab shuffleboardTab);

  @Override
  public boolean isFinished() {
    return getFailedTests().size() == 0;
  }

  @Override
  public void initialize() {
    getOutcomes().replaceAll((k, v) -> false);
    getTestCommand().withTimeout(SwerveModuleConstants.TEST_TIMEOUT).schedule();
  }

  protected List<String> getFailedTests() {
    return getOutcomes().entrySet().stream()
        .filter(entry -> !entry.getValue())
        .map(entry -> entry.getKey().toString())
        .collect(Collectors.toList());
  }
}

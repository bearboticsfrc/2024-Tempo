package frc.bearbotics.test;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public abstract class AbstractTestCommand extends Command {
  protected abstract Command getTestCommand();

  protected abstract Map<String, Boolean> getOutcomes();

  protected abstract double getTestTimeout();

  protected abstract void setupShuffleboardTab(ShuffleboardTab shuffleboardTab);

  @Override
  public boolean isFinished() {
    return getFailedTests().size() == 0;
  }

  @Override
  public void initialize() {
    getOutcomes().replaceAll((k, v) -> false);
    getTestCommand().withTimeout(getTestTimeout()).schedule();
  }

  protected void setPass(String name, boolean passed) {
    getOutcomes().put(name, passed);
  }

  protected List<String> getFailedTests() {
    return getOutcomes().entrySet().stream()
        .filter(entry -> !entry.getValue())
        .map(entry -> entry.getKey().toString())
        .collect(Collectors.toList());
  }
}

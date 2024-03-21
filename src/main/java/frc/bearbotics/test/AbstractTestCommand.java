package frc.bearbotics.test;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveModuleConstants;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * An abstract class for defining test commands within a robot's command-based system. This class
 * provides a structure for implementing tests that can be executed and monitored via the robot's
 * command framework and visualized through Shuffleboard. It allows for specifying individual test
 * commands, recording outcomes, and setting up a Shuffleboard tab for interaction and
 * visualization.
 */
public abstract class AbstractTestCommand extends Command {

  /**
   * Retrieves the specific Command instance representing the test to be executed. Implementing
   * classes must provide the logic to create and configure this command.
   *
   * @return The Command instance to be tested.
   */
  protected abstract Command getTestCommand();

  /**
   * Provides a map of test outcomes, where the key is a String identifying the test, and the value
   * is a Boolean indicating the test result (true for pass, false for fail). Implementing classes
   * must provide the logic for defining and updating these outcomes.
   *
   * @return A Map<String, Boolean> representing the outcomes of individual tests.
   */
  protected abstract Map<String, Boolean> getOutcomes();

  /**
   * Sets up a Shuffleboard tab for the test command. This method allows for customizing the
   * Shuffleboard interface to display test controls and outcomes.
   *
   * @param shuffleboardTab The ShuffleboardTab instance to be configured.
   */
  protected abstract void setupShuffleboardTab(ShuffleboardTab shuffleboardTab);

  /**
   * Determines if the command has completed its execution. In the context of this test command,
   * completion is defined by the absence of failed tests.
   *
   * @return true if there are no failed tests, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return getFailedTests().size() == 0;
  }

  /**
   * Initializes the test command by resetting the outcomes of all tests to false (fail) and
   * scheduling the test command with a predefined timeout from the SwerveModuleConstants.
   */
  @Override
  public void initialize() {
    getOutcomes().replaceAll((k, v) -> false);
    getTestCommand().withTimeout(SwerveModuleConstants.TEST_TIMEOUT).schedule();
  }

  /**
   * Retrieves a list of the names of tests that have failed. This is determined by filtering the
   * outcomes map for entries with a false value.
   *
   * @return A List<String> containing the keys (names) of failed tests.
   */
  protected List<String> getFailedTests() {
    return getOutcomes().entrySet().stream()
        .filter(entry -> !entry.getValue())
        .map(Map.Entry::getKey)
        .collect(Collectors.toList());
  }
}

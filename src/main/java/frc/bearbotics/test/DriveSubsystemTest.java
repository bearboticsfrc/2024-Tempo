package frc.bearbotics.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.bearbotics.swerve.SwerveModule;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A command to perform a series of automated tests on the DriveSubsystem to verify the
 * functionality of each swerve module's drive and pivot capabilities.
 */
public class DriveSubsystemTest extends AbstractTestCommand {
  private final DriveSubsystem driveSubsystem;

  /**
   * A map to record the outcomes of the tests, where the key is the test name (module and test
   * type) and the value is a boolean indicating whether the test passed (true) or failed (false).
   */
  private Map<String, Boolean> outcomes =
      new LinkedHashMap<String, Boolean>() {
        {
          put("FL Drive", false);
          put("FL Pivot", false);
          put("FR Drive", false);
          put("FR Pivot", false);
          put("BL Drive", false);
          put("BL Pivot", false);
          put("BR Drive", false);
          put("BR Pivot", false);
        }
      };

  /**
   * Constructs a new DriveSubsystemTest command.
   *
   * @param driveSubsystem The DriveSubsystem to be tested.
   * @param shuffleboardTab The ShuffleboardTab to display the test results.
   */
  public DriveSubsystemTest(DriveSubsystem driveSubsystem, ShuffleboardTab shuffleboardTab) {
    this.driveSubsystem = driveSubsystem;
    setupShuffleboardTab(shuffleboardTab);
  }

  @Override
  protected void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    int row = 0, column = 0;

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
    for (SwerveModule module : driveSubsystem.getSwerveModules()) {
      module.set(new SwerveModuleState(0, module.getRelativeAngle()));
    }

    List<String> failed = getFailedTests();

    if (failed.size() > 0) {
      DriverStation.reportError(
          "[Drive Subsystem Test] "
              + failed.size()
              + " motors failed test(s): "
              + String.join(", ", failed),
          false);
    }
  }

  @Override
  protected ParallelCommandGroup getTestCommand() {
    return new ParallelCommandGroup(
        driveSubsystem.getSwerveModules().stream()
            .map(this::getModuleTestCommand)
            .toArray(SequentialCommandGroup[]::new));
  }

  /**
   * Constructs a command to test a single swerve module. It checks both the drive and pivot
   * capabilities by setting a new state and verifying the module's response.
   *
   * @param module The SwerveModule to test.
   * @return A command sequence to test the specified module.
   */
  private Command getModuleTestCommand(SwerveModule module) {
    Rotation2d initialSteerAngle = module.getRelativeAngle();
    SwerveModuleState newState =
        new SwerveModuleState(0.5, Rotation2d.fromDegrees(90).plus(driveSubsystem.getHeading()));

    return Commands.sequence(
        Commands.runOnce(() -> module.set(newState)),
        Commands.waitSeconds(SwerveModuleConstants.TEST_WAIT),
        Commands.runOnce(() -> outcomes.put(module + " Drive", module.getDriveVelocity() != 0)),
        Commands.runOnce(
            () -> outcomes.put(module + " Pivot", module.getRelativeAngle() != initialSteerAngle)));
  }
}

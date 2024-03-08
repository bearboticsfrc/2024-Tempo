package frc.bearbotics.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.manipulator.ShooterConstants;
import frc.robot.subsystems.manipulator.ShooterSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterMotor;
import java.util.List;
import java.util.Map;

public class ShooterSubsystemTest extends AbstractTestCommand {
  private final String NAME = "Shooter Subsystem Test";

  private final ShooterSubsystem shooterSubsystem;

  private Map<String, Boolean> outcomes =
      Map.of("Upper Shooter Motor", false, "Lower Shooter Motor", false);

  public ShooterSubsystemTest(ShooterSubsystem shooterSubsystem, ShuffleboardTab shuffleboardTab) {
    this.shooterSubsystem = shooterSubsystem;

    setupShuffleboardTab(shuffleboardTab);
  }

  @Override
  protected double getTestTimeout() {
    return ShooterConstants.Test.TIMEOUT;
  }

  @Override
  protected void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    int row = 4, column = 0;

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
    shooterSubsystem.set(0);

    List<String> failed = getFailedTests();

    if (failed.size() > 0) {
      DriverStation.reportError(
          NAME + failed.size() + " motors failed test(s): " + String.join(", ", failed), false);
    }
  }

  @Override
  protected Command getTestCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.set(ShooterConstants.Test.VELOCITY)),
        Commands.waitSeconds(ShooterConstants.Test.WAIT),
        Commands.runOnce(
            () ->
                setPass(
                    "Upper Shooter Motor",
                    isPassing(shooterSubsystem.getVelocity(ShooterMotor.UPPER)))),
        Commands.runOnce(
            () ->
                setPass(
                    "Lower Shooter Motor",
                    isPassing(shooterSubsystem.getVelocity(ShooterMotor.LOWER)))));
  }

  private boolean isPassing(double velocity) {
    return MathUtil.isNear(
        ShooterConstants.Test.VELOCITY, velocity, ShooterConstants.Test.VELOCITY_TOLERANCE);
  }
}

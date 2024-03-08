package frc.bearbotics.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.manipulator.ArmConstants;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import java.util.List;
import java.util.Map;

public class ArmSubsystemTest extends AbstractTestCommand {
  private final String NAME = "Arm Subsystem Test";

  private final ArmSubsystem armSubsystem;

  private Map<String, Boolean> outcomes = Map.of("Arm Position", false);

  public ArmSubsystemTest(ArmSubsystem armSubsystem, ShuffleboardTab shuffleboardTab) {
    this.armSubsystem = armSubsystem;

    setupShuffleboardTab(shuffleboardTab);
  }

  @Override
  protected double getTestTimeout() {
    return ArmConstants.Test.TIMEOUT;
  }

  @Override
  protected void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    int row = 0, column = 6;

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
    armSubsystem.set(ArmPosition.HOME);

    List<String> failed = getFailedTests();

    if (failed.size() > 0) {
      DriverStation.reportError(
          NAME + failed.size() + " motor failed test(s): " + String.join(", ", failed), false);
    }
  }

  @Override
  protected Command getTestCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> armSubsystem.set(30)),
        Commands.waitSeconds(ArmConstants.Test.WAIT),
        Commands.runOnce(
            () -> setPass("Arm Motor", isPassing(armSubsystem.getPosition().getDegrees()))));
  }

  private boolean isPassing(double position) {
    return MathUtil.isNear(
        ArmConstants.Test.POSITION, position, ArmConstants.Test.POSITION_TOLERANCE);
  }
}

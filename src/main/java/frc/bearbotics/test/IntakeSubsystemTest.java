package frc.bearbotics.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.manipulator.IntakeConstants;
import frc.robot.subsystems.manipulator.IntakeSubsystem;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import java.util.List;
import java.util.Map;

public class IntakeSubsystemTest extends AbstractTestCommand {
  private final String NAME = "Intake Subsystem Test";

  private final IntakeSubsystem intakeSubsystem;

  private Map<String, Boolean> outcomes = Map.of("Intake Rollers", false, "Intake Feeder", false);

  public IntakeSubsystemTest(IntakeSubsystem intakeSubsystem, ShuffleboardTab shuffleboardTab) {
    this.intakeSubsystem = intakeSubsystem;

    setupShuffleboardTab(shuffleboardTab);
  }

  @Override
  protected double getTestTimeout() {
    return IntakeConstants.Test.TIMEOUT;
  }

  @Override
  protected void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    int row = 4, column = 9;

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
    intakeSubsystem.setRoller(IntakeSpeed.OFF);
    intakeSubsystem.setFeeder(IntakeSpeed.OFF);

    List<String> failed = getFailedTests();

    if (failed.size() > 0) {
      DriverStation.reportError(
          NAME + failed.size() + " motors failed test(s): " + String.join(", ", failed), false);
    }
  }

  @Override
  protected Command getTestCommand() {
    // TODO: Make more advanced (feed note and check beam breaks?)
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> intakeSubsystem.setRoller(IntakeConstants.Test.SPEED)),
            Commands.runOnce(() -> intakeSubsystem.setFeeder(IntakeConstants.Test.SPEED))),
        Commands.waitSeconds(IntakeConstants.Test.WAIT),
        Commands.runOnce(
            () -> setPass("Intake Rollers", isPass(intakeSubsystem.getRollerVelocity()))),
        Commands.runOnce(
            () -> setPass("Intake Feeder", isPass(intakeSubsystem.getFeederVelocity()))));
  }

  private boolean isPass(double velocity) {
    return MathUtil.isNear(
        IntakeConstants.Test.SPEED.getSpeed(),
        intakeSubsystem.getRollerVelocity(),
        IntakeConstants.Test.SPEED_TOLERANCE);
  }
}

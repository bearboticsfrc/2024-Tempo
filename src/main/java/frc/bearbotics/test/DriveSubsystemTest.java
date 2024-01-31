package frc.bearbotics.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class DriveSubsystemTest extends AbstractTestCommand {
  private final DriveSubsystem driveSubsystem;

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
      module.set(new SwerveModuleState(0, module.getSteerAngle()));
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

  private SequentialCommandGroup getModuleTestCommand(SwerveModule module) {
    Rotation2d initalSteerAngle = module.getSteerAngle();
    SwerveModuleState newState =
        new SwerveModuleState(0.5, Rotation2d.fromDegrees(90).plus(driveSubsystem.getHeading()));

    return new SequentialCommandGroup(
        new InstantCommand(() -> module.set(newState)),
        new WaitCommand(SwerveModuleConstants.TEST_WAIT),
        new InstantCommand(() -> outcomes.put(module + " Drive", module.getDriveVelocity() != 0)),
        new InstantCommand(
            () -> outcomes.put(module + " Pivot", module.getSteerAngle() != initalSteerAngle)));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    this(driveSubsystem, manipulatorSubsystem, () -> 0.0, () -> 0.0);
  }

  public AutoShootCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    addCommands(
        driveSubsystem.getDriveStopCommand(),
        Commands.parallel(
            new AutoAimCommand(driveSubsystem),
            getShooterPrepareCommand(manipulatorSubsystem, driveSubsystem)),
        manipulatorSubsystem.getShootCommand());

    addRequirements(driveSubsystem, manipulatorSubsystem);
  }

  private Command getShooterPrepareCommand(
      ManipulatorSubsystem manipulatorSubsystem, DriveSubsystem driveSubsystem) {
    return Commands.defer(
        () ->
            manipulatorSubsystem.getShooterPrepareCommad(
                () ->
                    LocationHelper.getDistanceToPose(
                        driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter())),
        Set.of(manipulatorSubsystem));
  }
}

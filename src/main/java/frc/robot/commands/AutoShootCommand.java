package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class AutoShootCommand extends SequentialCommandGroup {
  private final DriveSubsystem driveSubsystem;
  private final ManipulatorSubsystem manipulatorSubsystem;

  /**
   * Constructs the AutoShootCommand with the DriveSubsystem and ManipulatorSubsystem.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param manipulatorSubsystem The ManipulatorSubsystem instance control.
   */
  public AutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    this(driveSubsystem, manipulatorSubsystem, () -> 0.0, () -> 0.0);
  }

  /**
   * Constructs the AutoShootCommand with the DriveSubsystem, ManipulatorSubsystem, and optional X
   * and Y component suppliers for dynamic target points.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param manipulatorSubsystem The ManipulatorSubsystem instance.
   * @param xSupplier Supplier for the X component for dynamic driving while shooting.
   * @param ySupplier Supplier for the Y component for dynamic driving while shooting.
   */
  public AutoShootCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.driveSubsystem = driveSubsystem;
    this.manipulatorSubsystem = manipulatorSubsystem;

    addCommands(
        driveSubsystem.getDriveStopCommand(),
        new AutoAimCommand(driveSubsystem, FieldPositions.getInstance().getSpeakerTranslation())
            .alongWith(getShooterPrepareCommand()),
        manipulatorSubsystem.getShootCommand());

    addRequirements(driveSubsystem, manipulatorSubsystem);
  }

  /**
   * Retrieves the shooter preparation command based on the current robot pose and speaker center
   * position.
   *
   * @param manipulatorSubsystem The ManipulatorSubsystem instance for shooter and manipulator
   *     control.
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @return The shooter preparation command.
   */
  private Command getShooterPrepareCommand() {
    return manipulatorSubsystem.getShooterPrepareCommand(this::getDistanceToSpeakerCenter);
  }

  /**
   * Retrieves the current distance to the speaker's center.
   *
   * @return The distance, in meters.
   */
  private double getDistanceToSpeakerCenter() {
    return LocationHelper.getDistanceToPose(
        driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter());
  }
}

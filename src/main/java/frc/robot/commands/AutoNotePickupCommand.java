package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import java.util.Optional;

public class AutoNotePickupCommand extends SequentialCommandGroup {
  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  /**
   * Constructs the AutoNotePickupCommand.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem instance.
   */
  public AutoNotePickupCommand(
      DriveSubsystem driveSubsystem, ObjectDetectionSubsystem objectDetectionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSubsystem;

    addCommands(
        driveSubsystem.getDriveStopCommand(), getAutoNoteAimCommand(), getAutoNoteDriveCommand());
    addRequirements(driveSubsystem);
  }

  /**
   * Creates an AutoAimCommand instance to the nearest note point.
   *
   * @param point The target point for auto-aiming.
   * @return An AutoAimCommand instance.
   */
  private AutoAimCommand getAutoNoteAimCommand() {
    return new AutoAimCommand(driveSubsystem, getPointToNearestNote());
  }

  /**
   * Creates an AutoDriveCommand instance to the nearest note point.
   *
   * @return An AutoDriveCommand instance.
   */
  private AutoDriveCommand getAutoNoteDriveCommand() {
    return new AutoDriveCommand(driveSubsystem, getPointToNearestNote());
  }

  /**
   * Retrieves the target point for auto-aiming and autonomous driving, based on the pose to nearest
   * note information from the ObjectDetectionSubsystem.
   *
   * @return The target note point.
   */
  private Translation2d getPointToNearestNote() {
    Optional<Pose2d> notePose =
        objectDetectionSubsystem.getPoseToNearestNote(driveSubsystem.getPose());

    // TODO: This might be invalid since PhotonVision was not returning X or Y components.
    return notePose.isPresent()
        ? notePose.get().getTranslation()
        : driveSubsystem.getPose().getTranslation();
  }
}

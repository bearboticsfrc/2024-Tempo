package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.RobotConstants;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.subsystems.vision.StringFormatting;

public class NoteHuntCommand extends SequentialCommandGroup {
  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  public NoteHuntCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      ObjectDetectionSubsystem objectDetectionSystem) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSystem;

    RobotConstants.VISION_SYSTEM_TAB.addString(
        "Note Pose", () -> StringFormatting.poseToString(getPose()));

    addCommands(
        new PointAtNoteCommand(driveSubsystem, objectDetectionSystem),
        new ParallelCommandGroup(
            new DriveToPoseCommand(driveSubsystem, this::getPose)
                .until(manipulatorSubsystem::isNoteInRoller),
            manipulatorSubsystem.getIntakeCommand()));
    addRequirements(driveSubsystem, manipulatorSubsystem, objectDetectionSystem);
  }

  private Pose2d getPose() {
    double distance =
        objectDetectionSubsystem.hasNoteInView()
            ? getDistanceFromPitch(
                objectDetectionSubsystem.getTargetToNearestNote().get().getPitch())
            : 0;

    return LocationHelper.getPoseByDistanceAndAngleToPose(
        driveSubsystem.getPose(), distance, driveSubsystem.getHeading());
  }

  /**
   * Extrapolate distance from target pitch.
   *
   * @param pitch Pitch of target
   */
  private double getDistanceFromPitch(double pitch) {
    return (0.0296406 * pitch) + 0.599285;
  }
}

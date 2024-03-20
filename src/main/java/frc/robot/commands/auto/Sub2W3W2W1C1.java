package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub2W3W2W1C1 {
  public static final String NAME = "Sub2W3W2W1C1";
  private static final String AUTO_NAME = "Sub2W3NoteNoShoot";

  private static final PathPlannerPath W3_TO_W2_PATH = PathPlannerPath.fromPathFile("W3toW2Rotate");
  private static final PathPlannerPath W2_TO_W1_PATH = PathPlannerPath.fromPathFile("W2toW1Rotate");
  private static final PathPlannerPath W1_TO_C1_PATH = PathPlannerPath.fromPathFile("W1toC1");
  private static final PathPlannerPath C1_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C1toShoot");

  /**
   * Retrieves a composite Command that represents the full autonomous routine.
   *
   * @param driveSubsystem The DriveSubsystem required for path following commands.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem required for note detection.
   * @param manipulatorSubsystem The ManipulatorSubsystem required for shooting and note handling.
   * @return The entire autonomous routine command.
   */
  public static Command get(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return manipulatorSubsystem
        .getSubwooferShootCommand()
        .andThen(new PathPlannerAuto(AUTO_NAME))
        .andThen(getNotePickupAndAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, W3_TO_W2_PATH))
        .andThen(getNotePickupAndAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, W2_TO_W1_PATH))
        .andThen(getNotePickupAndAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, W1_TO_C1_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, C1_TO_SHOOT_PATH))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  /**
   * Composes a Command that waits for a note to be in the feeder, and then auto shoots.
   *
   * @param driveSubsystem The DriveSubsystem used in the AutoShootCommand.
   * @param manipulatorSubsystem The ManipulatorSubsystem that checks if note is in the feeder.
   * @return A Command object that combines waiting for a note and shooting.
   */
  private static Command getNotePickupAndAutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return Commands.waitUntil(manipulatorSubsystem::isNoteInFeeder)
        .andThen(new AutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  /**
   * Composes a Command that follows a replanned path and prepares the shooter.
   *
   * @param driveSubsystem The DriveSubsystem used for replanning the path.
   * @param manipulatorSubsystem The ManipulatorSubsystem to prepare the shooter.
   * @param path The PathPlannerPath that defines the initial path to follow.
   * @return A Command object that executes the path following and shooter preparation.
   */
  private static Command getReplannedPathAndShooterPrepareCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      PathPlannerPath path) {
    PathPlannerPath replannedPath =
        path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());

    return AutoBuilder.followPath(replannedPath)
        .alongWith(manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT));
  }

  /**
   * Composes the auto-note pickup command.
   *
   * @param driveSubsystem The DriveSubsystem required for the AutoNotePickupCommand.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem used to detect notes on the field.
   * @param manipulatorSubsystem The ManipulatorSubsystem used for intake operations.
   * @return The auto-note pickup command.
   */
  private static Command getAutoNotePickupCommand(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return driveSubsystem
        .getDriveStopCommand()
        .andThen(
            new AutoNotePickupCommand(
                    driveSubsystem,
                    objectDetectionSubsystem,
                    () -> manipulatorSubsystem.isNoteInIntake())
                .alongWith(manipulatorSubsystem.getIntakeCommand()));
  }

  /**
   * Composes the auto shoot command.
   *
   * @param driveSubsystem The DriveSubsystem required for positioning during the shooting.
   * @param manipulatorSubsystem The ManipulatorSubsystem that controls the shooting mechanism.
   * @return A Command object that when executed will auto shoot.
   */
  private static Command getAutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new AutoShootCommand(driveSubsystem, manipulatorSubsystem);
  }
}

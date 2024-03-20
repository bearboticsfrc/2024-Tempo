package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.bearbotics.campaign.Campaign;
import frc.bearbotics.campaign.CampaignExecutor;
import frc.bearbotics.campaign.CommandMission;
import frc.bearbotics.campaign.MissionTree;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class SmartSub3ToC5C3 {
  public static final String NAME = "SmartSub3ToC5C3";

  private static final PathPlannerPath C5_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C5ToShoot");

  private static final PathPlannerPath SUB3_TO_C5 = PathPlannerPath.fromPathFile("Sub3ToC5");

  private static final PathPlannerPath C5_TO_C4_PATH = PathPlannerPath.fromPathFile("C5ToC4");
  private static final PathPlannerPath C4_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C4toShoot");

  private static final PathPlannerPath SHOOT_TO_C3_PATH = PathPlannerPath.fromPathFile("ShootToC3");
  private static final PathPlannerPath C3_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C3ToShoot");

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
      ManipulatorSubsystem manipulatorSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem) {
    return getAutoShootCommand(driveSubsystem, manipulatorSubsystem)
        .andThen(AutoBuilder.followPath(SUB3_TO_C5))
        .andThen(getC5C4Campaign(driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, SHOOT_TO_C3_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, C3_TO_SHOOT_PATH))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  /**
   * This campaign is as follows:
   *
   * <p>C5 Pickup:
   *
   * <ul>
   *   <li>On success: Path to shooting location.
   *   <li>On failure: Path to C4 and pickup.
   * </ul>
   *
   * <p>C4 Pickup:
   *
   * <ul>
   *   <li>On success or failure: Path to shooting location.
   * </ul>
   *
   * <br>
   *
   * @param driveSubsystem The DriveSubsystem used for movement and path following.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem for detecting objects to be picked
   *     up.
   * @param manipulatorSubsystem The ManipulatorSubsystem responsible for object handling and
   *     shooting.
   * @return A Command that represents the campaign from C5 to C4. Dynamically makes decisions based
   *     on mission outcomes.
   */
  private static Command getC5C4Campaign(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    CommandMission c5PickupMission =
        new CommandMission(
                driveSubsystem
                    .getDriveStopCommand()
                    .andThen(
                        getAutoNotePickupCommand(
                            driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
                    .withTimeout(1)
                    .withName("C5 Pickup"))
            .withSuccessCallback(manipulatorSubsystem::isNoteInIntake);

    CommandMission c5ToShootMission =
        new CommandMission(
            getReplannedPathAndShooterPrepareCommand(
                    driveSubsystem, manipulatorSubsystem, C5_TO_SHOOT_PATH)
                .withName("C5 To Shoot"));

    // TODO: Maybe add precondition here?
    CommandMission c5ToC4PickupMission =
        new CommandMission(
                getReplannedPathAndShooterPrepareCommand(
                        driveSubsystem, manipulatorSubsystem, C5_TO_C4_PATH)
                    .andThen(
                        getAutoNotePickupCommand(
                            driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
                    .withName("C5 To C4 Pickup"))
            .withSuccessCallback(manipulatorSubsystem::isNoteInIntake);

    CommandMission c4ToShootMission =
        new CommandMission(
            AutoBuilder.followPath(C4_TO_SHOOT_PATH)
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT))
                .withName("C4 To Shoot"));

    MissionTree c5ToC4Mission =
        new MissionTree(c5ToC4PickupMission).withSuccessNode(c4ToShootMission);

    MissionTree c5Mission =
        new MissionTree(c5PickupMission)
            .withSuccessNode(c5ToShootMission)
            .withFailureNode(c5ToC4Mission);

    Campaign campaign = new Campaign(NAME, c5Mission);
    return new CampaignExecutor(campaign);
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
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem::isNoteInIntake))
        .alongWith(manipulatorSubsystem.getIntakeCommand());
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

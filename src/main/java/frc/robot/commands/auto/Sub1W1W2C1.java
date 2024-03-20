package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub1W1W2C1 {
  public static final String NAME = "Sub1W1W2C1-MaybeC2";

  private static final PathPlannerPath W1_TO_W2_PATH = PathPlannerPath.fromPathFile("W1ToW2");

  private static final PathPlannerPath W2_TO_C1_PATH =
      PathPlannerPath.fromPathFile("W2ToC1Version2");
  private static final PathPlannerPath C1_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C1toShoot");
  private static final PathPlannerPath C2_TO_SHOOT_PATH = PathPlannerPath.fromPathFile("C2ToShoot");
  private static final PathPlannerPath SHOOT_TO_C2_PATH = PathPlannerPath.fromPathFile("ShootToC2");

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
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(AutoBuilder.followPath(W1_TO_W2_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, W2_TO_C1_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, C1_TO_SHOOT_PATH))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, SHOOT_TO_C2_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(AutoBuilder.followPath(C2_TO_SHOOT_PATH))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
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
                    () ->
                        manipulatorSubsystem.isNoteInFeeder()
                            || manipulatorSubsystem.isNoteInRoller())
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

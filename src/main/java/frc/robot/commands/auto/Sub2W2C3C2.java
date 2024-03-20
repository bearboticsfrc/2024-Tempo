package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub2W2C3C2 {
  public static final String NAME = "Sub2W2C3C2";
  private static final String AUTO_NAME = "Sub2W2";
  private static PathPlannerPath W2_TO_C3_PATH = PathPlannerPath.fromPathFile("W2toC3");
  private static PathPlannerPath C3_TO_SHOOT_PATH =
      PathPlannerPath.fromPathFile("C3ToShootVersion2");
  private static PathPlannerPath SHOOT_TO_C2_PATH =
      PathPlannerPath.fromPathFile("ShootToC2AfterC3");
  private static PathPlannerPath C2_TO_SHOOT_PATH =
      PathPlannerPath.fromPathFile("C2ToShootAfterC3");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return new PathPlannerAuto(AUTO_NAME)
        .alongWith(manipulatorSubsystem.getLineShootCommand())
        .andThen(new WaitUntilCommand(manipulatorSubsystem::isNoteInFeeder))
        .andThen(manipulatorSubsystem.getFarPodiumShootCommand())
        .andThen(getReplannedPath(driveSubsystem, W2_TO_C3_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            getReplannedPathAndShooterPrepareCommand(
                driveSubsystem, manipulatorSubsystem, C3_TO_SHOOT_PATH))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(getReplannedPath(driveSubsystem, SHOOT_TO_C2_PATH))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(C2_TO_SHOOT_PATH)
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
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
   * Composes a Command that follows a replanned path.
   *
   * @param driveSubsystem The DriveSubsystem used for replanning the path.
   * @param path The PathPlannerPath that defines the initial path to follow.
   * @return A Command object that executes the path following and shooter preparation.
   */
  private static Command getReplannedPath(DriveSubsystem driveSubsystem, PathPlannerPath path) {
    PathPlannerPath replannedPath =
        path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());

    return AutoBuilder.followPath(replannedPath);
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
}

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub3ToC5C3 {
  public static final String NAME = "Sub3ToC5C3";

  private static final PathPlannerPath c5ToShootPath = PathPlannerPath.fromPathFile("C5ToShoot");
  private static final PathPlannerPath shootToC3 = PathPlannerPath.fromPathFile("ShootToC3");
  private static final PathPlannerPath c3ToShoot = PathPlannerPath.fromPathFile("C3ToShoot");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem) {
    return getAutoShootCommand(driveSubsystem, manipulatorSubsystem)
        .andThen(new PathPlannerAuto(NAME))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedC5ToShootPath(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(AutoBuilder.followPath(getReplannedShootToC3Path(driveSubsystem)))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedC3ToShootPath(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  private static Command getAutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new AutoShootCommand(driveSubsystem, manipulatorSubsystem);
  }

  private static Command getAutoNotePickupCommand(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return new AutoNotePickupCommand(
            driveSubsystem,
            objectDetectionSubsystem,
            () -> manipulatorSubsystem.isNoteInFeeder() || manipulatorSubsystem.isNoteInRoller())
        .alongWith(manipulatorSubsystem.getIntakeCommand());
  }

  private static PathPlannerPath getReplannedC5ToShootPath(DriveSubsystem driveSubsystem) {
    return c5ToShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedShootToC3Path(DriveSubsystem driveSubsystem) {
    return shootToC3.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedC3ToShootPath(DriveSubsystem driveSubsystem) {
    return c3ToShoot.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }
}

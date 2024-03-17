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
  private static final String AUTO_NAME = "Sub2W2";

  public static final String NAME = "Sub2W2C3C4";

  static PathPlannerPath w2toc3Path = PathPlannerPath.fromPathFile("W2toC3");
  static PathPlannerPath c3toShootPath = PathPlannerPath.fromPathFile("C3ToShoot");

  static PathPlannerPath shootToc2Path = PathPlannerPath.fromPathFile("ShootToC2");

  static PathPlannerPath c2toShootPath = PathPlannerPath.fromPathFile("C2ToShoot");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return new PathPlannerAuto(AUTO_NAME)
        .alongWith(manipulatorSubsystem.getFarLineShootCommand())
        .andThen(new WaitUntilCommand(manipulatorSubsystem::isNoteInFeeder))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(AutoBuilder.followPath(getReplannedW2ToC3Path(driveSubsystem)))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedC3ToShootPath(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(AutoBuilder.followPath(getReplannedShootToC2Path(driveSubsystem)))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedC2ToShootPath(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  private static Command getAutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new AutoShootCommand(driveSubsystem, manipulatorSubsystem);
  }

  private static PathPlannerPath getReplannedW2ToC3Path(DriveSubsystem driveSubsystem) {
    return w2toc3Path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedC3ToShootPath(DriveSubsystem driveSubsystem) {
    return c3toShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedShootToC2Path(DriveSubsystem driveSubsystem) {
    return shootToc2Path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedC2ToShootPath(DriveSubsystem driveSubsystem) {
    return c2toShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
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
}

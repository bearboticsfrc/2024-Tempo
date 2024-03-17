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

public class Sub2W3W2W1C1 {
  public static final String NAME = "Sub2W3W2W1C1";

  static PathPlannerPath w3tow2Path = PathPlannerPath.fromPathFile("W3toW2Rotate");
  static PathPlannerPath w2tow1Path = PathPlannerPath.fromPathFile("W2toW1Rotate");
  static PathPlannerPath w1toc1Path = PathPlannerPath.fromPathFile("W1toC1");
  static PathPlannerPath c1toShootPath = PathPlannerPath.fromPathFile("C1toShoot");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    return manipulatorSubsystem
        .getSubwooferShootCommand()
        .andThen(new PathPlannerAuto("Sub2W3NoteNoShoot"))
        .andThen(new WaitUntilCommand(manipulatorSubsystem::isNoteInFeeder))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedW3ToW2Path(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(new WaitUntilCommand(manipulatorSubsystem::isNoteInFeeder))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedW2ToW1Path(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(new WaitUntilCommand(manipulatorSubsystem::isNoteInFeeder))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedW1ToC1Path(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(
            getAutoNotePickupCommand(
                driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem))
        .andThen(
            AutoBuilder.followPath(getReplannedC1ToShootPath(driveSubsystem))
                .alongWith(
                    manipulatorSubsystem.getShooterPrepareCommand(ShooterVelocity.PODIUM_SHOOT)))
        .andThen(getAutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  private static Command getAutoShootCommand(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new AutoShootCommand(driveSubsystem, manipulatorSubsystem);
  }

  private static PathPlannerPath getReplannedW3ToW2Path(DriveSubsystem driveSubsystem) {
    return w3tow2Path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedW2ToW1Path(DriveSubsystem driveSubsystem) {
    return w2tow1Path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedW1ToC1Path(DriveSubsystem driveSubsystem) {
    return w1toc1Path.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }

  private static PathPlannerPath getReplannedC1ToShootPath(DriveSubsystem driveSubsystem) {
    return c1toShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
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

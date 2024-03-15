package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub3ToC5 {
  private static PathPlannerPath c5ToShootPath = PathPlannerPath.fromPathFile("C5ToShoot");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem) {
    return new AutoAimCommand(driveSubsystem, FieldPositions.getInstance().getSpeakerTranslation())
        .andThen(
            manipulatorSubsystem.getArmAndShooterPrepareCommand(
                () ->
                    LocationHelper.getDistanceToPose(
                        driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter())))
        .andThen(manipulatorSubsystem.getShootCommand())
        .andThen(new PathPlannerAuto("Sub3ToC5"))
        .andThen(
            new AutoNotePickupCommand(
                    driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem::isNoteInRoller)
                .alongWith(manipulatorSubsystem.getIntakeCommand()))
        .andThen(Commands.waitUntil(manipulatorSubsystem::isNoteInFeeder))
        .andThen(Commands.runOnce(() -> replanC5ToShootPath(driveSubsystem)))
        .andThen(AutoBuilder.followPath(c5ToShootPath))
        .andThen(
            new AutoAimCommand(
                driveSubsystem, FieldPositions.getInstance().getSpeakerTranslation()))
        .andThen(
            manipulatorSubsystem
                .getArmAndShooterPrepareCommand(
                    () ->
                        LocationHelper.getDistanceToPose(
                            driveSubsystem.getPose(),
                            FieldPositions.getInstance().getSpeakerCenter()))
                .andThen(manipulatorSubsystem.getShootCommand()));
  }

  private static void replanC5ToShootPath(DriveSubsystem driveSubsystem) {
    c5ToShootPath =
        c5ToShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }
}

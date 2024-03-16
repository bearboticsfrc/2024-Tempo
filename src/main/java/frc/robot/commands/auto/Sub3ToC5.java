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
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class Sub3ToC5 {
  private static PathPlannerPath c5ToShootPath = PathPlannerPath.fromPathFile("C5ToShoot");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem) {
    return new AutoShootCommand(driveSubsystem, manipulatorSubsystem)
        .andThen(new PathPlannerAuto("Sub3ToC5"))
        .andThen(
            new AutoNotePickupCommand(
                    driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem::isNoteInRoller)
                .alongWith(manipulatorSubsystem.getIntakeCommand()))
        .andThen(Commands.waitUntil(manipulatorSubsystem::isNoteInFeeder))
        .andThen(AutoBuilder.followPath(getReplannedC5ToShootPath(driveSubsystem)))
        .andThen(new AutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }

  private static PathPlannerPath getReplannedC5ToShootPath(DriveSubsystem driveSubsystem) {
    return c5ToShootPath.replan(driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
  }
}

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoShootCommand;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.Set;

public class FourNoteAmp {
  static final String NAME = "Sub1ToW1";

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulator) {

    PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(NAME);
    PathPlannerPath w1toc1Path = PathPlannerPath.fromPathFile("W1toC1");

    return Commands.sequence(
            manipulator.getSubwooferShootCommand(),
            AutoBuilder.followPath(pathPlannerPath),
            manipulator.getIntakeCommand(),
            Commands.waitUntil(manipulator::isNoteInFeeder),
            manipulator.getAutoShootCommand(
                () ->
                    LocationHelper.getDistanceToPose(
                        driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter())),
            AutoBuilder.followPath(w1toc1Path),
            Commands.defer(
                () -> new AutoShootCommand(driveSubsystem, manipulator),
                Set.of(driveSubsystem, manipulator)))
        .withName("FourNoteAmp");
  }
}

package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.bearbotics.campaign.Campaign;
import frc.bearbotics.campaign.CommandMission;
import frc.bearbotics.campaign.MissionTree;
import frc.robot.commands.auto.missions.PathCommandMission;
import frc.robot.fms.AllianceColor;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;

public class ExitCommunityRight {

  public static final String NAME = "Shoot Note Exit Community Right";
  private BooleanSupplier isBlue = () -> (AllianceColor.alliance == Alliance.Blue) ? true : false;

  public static Campaign get(DriveSubsystem driveSubsystem) {

    final MissionTree pathMissionNode = new MissionTree(getPathMission(driveSubsystem));

    return new Campaign(NAME, pathMissionNode);
  }

  public static CommandMission getPathMission(DriveSubsystem driveSubsystem) {
    PathPlannerPath path = PathPlannerPath.fromPathFile("ShootNoteExitCommunityRight");

    return new CommandMission(
        new SequentialCommandGroup(
                new PathCommandMission(driveSubsystem, path), driveSubsystem.getDriveStopCommand())
            .withName("ShootNoteExitCommunityRight"));
  }
}

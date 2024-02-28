package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.Set;

public class MiddleC1 {
  static PathPlannerPath midToC2 = PathPlannerPath.fromPathFile("MidShoottoC2NoShoot");

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem, boolean tryC2) {
    return new SequentialCommandGroup(
        manipulatorSubsystem.getSubwooferShootCommand(),
        new PathPlannerAuto("MiddleC1"),
        new DeferredCommand(
            () -> new AutoShootCommand(driveSubsystem, manipulatorSubsystem),
            Set.of(driveSubsystem)),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new DeferredCommand(
                    () ->
                        new InstantCommand(
                            () -> {
                              midToC2 =
                                  midToC2.replan(
                                      driveSubsystem.getPose(),
                                      driveSubsystem.getRobotRelativeSpeeds());
                            }),
                    Set.of()),
                AutoBuilder.followPath(midToC2),
                new DeferredCommand(
                    () -> new AutoShootCommand(driveSubsystem, manipulatorSubsystem),
                    Set.of(driveSubsystem))),
            new InstantCommand(),
            () -> tryC2));
  }
}

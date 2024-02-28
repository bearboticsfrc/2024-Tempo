package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class Sub1TwoNote {
  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
    return new SequentialCommandGroup(
        manipulatorSubsystem.getSubwooferShootCommand(),
        new PathPlannerAuto("Sub1TwoNote"),
        Commands.waitUntil(manipulatorSubsystem::isNoteInFeeder),
        new AutoShootCommand(driveSubsystem, manipulatorSubsystem));
  }
}

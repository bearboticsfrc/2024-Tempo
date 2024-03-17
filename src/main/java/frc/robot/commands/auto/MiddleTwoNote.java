package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleTwoNote {
  public static String NAME = "MiddleTwoNote";

  public static Command get(ManipulatorSubsystem manipulatorSubsystem) {
    return new SequentialCommandGroup(
        manipulatorSubsystem.getSubwooferShootCommand(), new PathPlannerAuto(NAME));
  }
}

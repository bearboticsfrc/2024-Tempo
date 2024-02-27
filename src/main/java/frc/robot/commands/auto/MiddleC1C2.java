package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleC1C2 {
  public static Command get(ManipulatorSubsystem manipulatorSubsystem) {
    return Commands.sequence(
        manipulatorSubsystem.getSubwooferShootCommand(), new PathPlannerAuto("MiddleC1C2"));
  }
}

package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class Sub1TwoNote {
  public static Command get(ManipulatorSubsystem manipulatorSubsystem) {
    return Commands.sequence(
        manipulatorSubsystem.getSubwooferShootCommand(), new PathPlannerAuto("Sub1TwoNote"));
  }
}

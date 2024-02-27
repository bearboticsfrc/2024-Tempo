package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class Sub3W3W2W1 {
  public static Command get(ManipulatorSubsystem manipulatorSubsystem) {
    return Commands.sequence(
        manipulatorSubsystem.getSubwooferShootCommand(), new PathPlannerAuto("W3W2W1"));
  }
}

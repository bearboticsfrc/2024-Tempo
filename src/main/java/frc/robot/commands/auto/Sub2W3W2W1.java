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
import frc.robot.subsystems.manipulator.ShooterSubsystem.ShooterVelocity;

import java.util.Set;

public class Sub2W3W2W1 {

  static PathPlannerPath w3tow2Path = PathPlannerPath.fromPathFile("W3toW2NoShoot");
  static PathPlannerPath w2tow1Path = PathPlannerPath.fromPathFile("W2toW1NoShoot");
  static PathPlannerPath w1toc1Path = PathPlannerPath.fromPathFile("W1toC1");

  public static Command get(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      boolean plusC1) {
    return new SequentialCommandGroup(
        manipulatorSubsystem.getSubwooferShootCommand()
        .andThen(manipulatorSubsystem.getShooterRunCommand(ShooterVelocity.PODIUM_SHOOT)),
        new PathPlannerAuto("Sub2W3NoteNoShoot"),
        new DeferredCommand(
            () -> new AutoShootCommand(driveSubsystem, manipulatorSubsystem)
            .andThen(manipulatorSubsystem.getShooterRunCommand(ShooterVelocity.PODIUM_SHOOT)),
            Set.of(driveSubsystem))
     ,
        new DeferredCommand(
            () ->
                new InstantCommand(
                    () -> {
                      w3tow2Path =
                          w3tow2Path.replan(
                              driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
                    }),
            Set.of()),
        AutoBuilder.followPath(w3tow2Path),
        new DeferredCommand(
            () -> new AutoShootCommand(driveSubsystem, manipulatorSubsystem),
            Set.of(driveSubsystem)),
        new DeferredCommand(
            () ->
                new InstantCommand(
                    () -> {
                      w2tow1Path =
                          w2tow1Path.replan(
                              driveSubsystem.getPose(), driveSubsystem.getRobotRelativeSpeeds());
                    }),
            Set.of()),
        AutoBuilder.followPath(w2tow1Path),
        new DeferredCommand(
            () -> new AutoShootCommand(driveSubsystem, manipulatorSubsystem),
            Set.of(driveSubsystem)),
        new ConditionalCommand(
            new DeferredCommand(
                    () ->
                        new InstantCommand(
                            () -> {
                              w1toc1Path =
                                  w1toc1Path.replan(
                                      driveSubsystem.getPose(),
                                      driveSubsystem.getRobotRelativeSpeeds());
                            }),
                    Set.of())
                .andThen(AutoBuilder.followPath(w1toc1Path)),
            new InstantCommand(),
            () -> plusC1)
            );
  }
}

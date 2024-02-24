package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.function.DoubleSupplier;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this(driveSubsystem, manipulatorSubsystem, poseEstimatorSubsystem, () -> 0.0, () -> 0.0);
  }

  public AutoShootCommand(
      DriveSubsystem driveSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    addCommands(
        new AutoAimCommand(driveSubsystem, poseEstimatorSubsystem, xSupplier, ySupplier),
        getAutoShootCommand(manipulatorSubsystem, poseEstimatorSubsystem));
  }

  private Command getAutoShootCommand(
      ManipulatorSubsystem manipulatorSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    return manipulatorSubsystem.getAutoShootCommand(
        () ->
            LocationHelper.getDistanceToPose(
                poseEstimatorSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter()));
  }
}

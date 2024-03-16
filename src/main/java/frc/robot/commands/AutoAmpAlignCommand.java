package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAmpAlignCommand extends SequentialCommandGroup {
  public AutoAmpAlignCommand(DriveSubsystem driveSubsystem) {
    Pose2d ampPose = FieldPositions.getInstance().getAmp();
    Translation2d ampTranslation = ampPose.getTranslation();

    addCommands(
        new AutoAimCommand(driveSubsystem, ampTranslation),
        new DriveToPoseCommand(driveSubsystem, ampPose));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;

  private DoubleSupplier xSupplier = () -> 0.0;
  private DoubleSupplier ySupplier = () -> 0.0;

  public AutoAimCommand(
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this(driveSubsystem, poseEstimatorSubsystem);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  public AutoAimCommand(
      DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    driveSubsystem.aimAtPoint(
        ySupplier,
        xSupplier,
        poseEstimatorSubsystem::getPose,
        FieldPositions.getInstance().getSpeakerTranslation());
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.atAimSetpoint();
  }
}

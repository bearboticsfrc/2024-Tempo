package frc.robot.commands.notehuntcommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RunAtNoteCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final PIDController ySpeedController = new PIDController(0.1, 0, 0);
  double distance;

  public RunAtNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (visionSubsystem.getNote() == null) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    PhotonTrackedTarget note = visionSubsystem.getNote();
    distance = note.getBestCameraToTarget().getX();
    Rotation2d rotation = Rotation2d.fromDegrees(note.getYaw());

    Pose2d desiredPose =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            driveSubsystem.getPose(), distance, rotation);
    DriveToPoseCommand driveToPoseCommand = new DriveToPoseCommand(desiredPose, driveSubsystem);
    driveToPoseCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return (ySpeedController.atSetpoint());
  }
}

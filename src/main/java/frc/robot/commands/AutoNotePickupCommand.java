package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoNotePickupCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  private final PIDController yawPidController = new PIDController(0.002, 0, 0);
  private final PIDController pitchPidController = new PIDController(0.002, 0, 0);

  /**
   * Constructs the AutoNotePickupCommand.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem instance.
   */
  public AutoNotePickupCommand(
      DriveSubsystem driveSubsystem, ObjectDetectionSubsystem objectDetectionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSubsystem;
  }

  @Override
  public void execute() {
    if (!objectDetectionSubsystem.hasNoteInView()) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    PhotonTrackedTarget target = objectDetectionSubsystem.getTargetToNearestNote().get();

    double rot = yawPidController.calculate(target.getYaw(), 0);
    double xSpeed = pitchPidController.calculate(target.getYaw(), 0);

    driveSubsystem.drive(xSpeed, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return yawPidController.atSetpoint() && pitchPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}

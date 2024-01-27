package frc.robot.commands.notehuntcommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PointAtNoteCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final PIDController rotSpeedController = new PIDController(0.01, 0, 0.001);

  public PointAtNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
  }

  @Override
  public void execute() {
    if (visionSubsystem.getNote() == null) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    Transform3d note = visionSubsystem.getNote().getBestCameraToTarget();

    Rotation2d targetRote = note.getRotation().toRotation2d();
    double rot = targetRote.getDegrees();

    rot = rotSpeedController.calculate(rot, 0);

    driveSubsystem.drive(0, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return (rotSpeedController.atSetpoint());
  }
}

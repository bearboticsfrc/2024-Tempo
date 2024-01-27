package frc.robot.commands.notehuntcommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RunAtNoteCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final PIDController ySpeedController = new PIDController(0.1, 0, 0);

  public RunAtNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
    ySpeedController.setTolerance(2);
  }

  @Override
  public void execute() {
    if (visionSubsystem.getNote() == null) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    Transform3d note = visionSubsystem.getNote().getBestCameraToTarget();
    double targetY = note.getY();

    double ySpeed = ySpeedController.calculate(targetY, 0);

    driveSubsystem.drive(0, ySpeed, 0, false);
  }

  @Override
  public boolean isFinished() {
    return (ySpeedController.atSetpoint());
  }
}

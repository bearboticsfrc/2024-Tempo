package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignSpeakerCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final PIDController ySpeedController = new PIDController(0.05, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.05, 0, 0);

  private final FieldPositions fieldPositions = new FieldPositions();

  public AlignSpeakerCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);

    ySpeedController.setTolerance(0.1);
    xSpeedController.setTolerance(0.1);
  }

  @Override
  public void execute() {

    if (visionSubsystem.getTarget(fieldPositions.getSpeakerCenterTagID()).isPresent()) {
      PhotonTrackedTarget speaker =
          visionSubsystem.getTarget(fieldPositions.getSpeakerCenterTagID()).get();
      double x = speaker.getBestCameraToTarget().getX();
      double y = speaker.getBestCameraToTarget().getY();
      double ySpeed = ySpeedController.calculate(x, 0);
      double xSpeed = xSpeedController.calculate(y, 0);

      driveSubsystem.drive(-xSpeed, ySpeed, 0, false);
      return;
    }

    driveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return ySpeedController.atSetpoint() && xSpeedController.atSetpoint();
  }
}

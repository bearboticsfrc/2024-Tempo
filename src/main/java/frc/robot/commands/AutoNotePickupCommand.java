package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoNotePickupCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  private final PIDController yawPidController = new PIDController(0.005, 0, 0.0001);
  private final PIDController pitchPidController = new PIDController(0.05, 0, 0);

  private final boolean yawMode;

  private boolean atSetpoint = false;

  private final Debouncer yawDebouncer = new Debouncer(.25);

  /**
   * Constructs the AutoNotePickupCommand.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem instance.
   */
  public AutoNotePickupCommand(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      boolean yawMode) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.yawMode = yawMode;
    yawPidController.setTolerance(1.0);
    pitchPidController.setTolerance(2.0);
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    //    if (!objectDetectionSubsystem.hasNoteInView()) {
    //      driveSubsystem.drive(0, 0, 0);
    //      return;
    //    }

    Optional<PhotonTrackedTarget> nearestNote = objectDetectionSubsystem.getTargetToNearestNote();

    if (nearestNote.isPresent()) {
      PhotonTrackedTarget target = nearestNote.get();

      double adjustedPitch = (target.getPitch() + 18.0) * -1.0;
      double rot = yawPidController.calculate(target.getYaw(), 0);
      double xSpeed = pitchPidController.calculate(adjustedPitch, 0);

      if (!yawMode) rot = 0.0;
      if (yawMode) xSpeed = 0.0;

      System.out.println(
          "yaw = "
              + String.format("%.02f", target.getYaw())
              + " pitch="
              + String.format("%.02f", adjustedPitch)
              + " rot = "
              + String.format("%.02f", rot)
              + " xSpeed = "
              + String.format("%.02f", xSpeed));

      xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

      driveSubsystem.drive(xSpeed, 0, rot, false);
      atSetpoint = yawDebouncer.calculate(yawPidController.atSetpoint());
    } else {
      System.out.println("not note!");
    }
  }

  @Override
  public boolean isFinished() {
    if (yawMode) {
      System.out.println(" error = " +String.format("%.02f",yawPidController.getPositionError()));
      return atSetpoint;
    }
    return pitchPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping!!!!!!");
    driveSubsystem.drive(0, 0, 0);
  }
}

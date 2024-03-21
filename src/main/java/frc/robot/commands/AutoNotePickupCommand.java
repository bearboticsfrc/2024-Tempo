package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.commands.AutoNotePickupConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Command to autonomously navigate the robot to pick up a note based on vision. */
public class AutoNotePickupCommand extends Command {
  private final double PITCH_OFFSET = 18;

  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  private final PIDController yawPidController;
  private final PIDController pitchPidController;

  private final BooleanSupplier hasNote;

  /**
   * Constructs the AutoNotePickupCommand.
   *
   * @param driveSubsystem The robot's drive subsystem, responsible for its movement.
   * @param objectDetectionSubsystem The vision subsystem used to detect and track the target note.
   * @param hasNote A BooleanSupplier indicating whether the note is successfully picked up.
   */
  public AutoNotePickupCommand(
      DriveSubsystem driveSubsystem,
      ObjectDetectionSubsystem objectDetectionSubsystem,
      BooleanSupplier hasNote) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.hasNote = hasNote;

    this.yawPidController =
        new PIDController(
            AutoNotePickupConstants.YawPid.P,
            AutoNotePickupConstants.YawPid.I,
            AutoNotePickupConstants.YawPid.D);
    this.pitchPidController =
        new PIDController(
            AutoNotePickupConstants.PitchPid.P,
            AutoNotePickupConstants.PitchPid.I,
            AutoNotePickupConstants.PitchPid.D);

    yawPidController.setTolerance(AutoNotePickupConstants.YawPid.TOLERANCE);
    pitchPidController.setTolerance(AutoNotePickupConstants.PitchPid.TOLERANCE);

    addRequirements(driveSubsystem, objectDetectionSubsystem);
  }

  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> maybeTarget = objectDetectionSubsystem.getBestTarget();
    if (!maybeTarget.isPresent()) {
      return;
    }

    PhotonTrackedTarget target = maybeTarget.get();
    double adjustedPitch = -(target.getPitch() + PITCH_OFFSET);
    double rotSpeed = yawPidController.calculate(target.getYaw());
    double xSpeed = (rotSpeed < 5) ? pitchPidController.calculate(adjustedPitch) : 0;

    driveSubsystem.drive(MathUtil.clamp(xSpeed, -0.5, 0.5), 0, rotSpeed, false);
  }

  @Override
  public boolean isFinished() {
    return hasNote.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class PointAtNoteCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  private final PIDController rotSpeedController = new PIDController(0.0021, 0, 0);

  public PointAtNoteCommand(
      DriveSubsystem driveSubsystem, ObjectDetectionSubsystem objectDetectionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.objectDetectionSubsystem = objectDetectionSubsystem;

    RobotConstants.VISION_SYSTEM_TAB.add("PAT - Rot", rotSpeedController);

    rotSpeedController.setTolerance(2);
    addRequirements(driveSubsystem, objectDetectionSubsystem);
  }

  @Override
  public void execute() {
    if (!objectDetectionSubsystem.hasNoteInView()) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double targetYaw = objectDetectionSubsystem.getTargetToNearestNote().get().getYaw();
    driveSubsystem.drive(0, 0, rotSpeedController.calculate(targetYaw), false);
  }

  @Override
  public boolean isFinished() {
    return rotSpeedController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}

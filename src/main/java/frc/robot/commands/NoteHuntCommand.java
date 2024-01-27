package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class NoteHuntCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final PIDController rotSpeedController = new PIDController(0.01, 0, 0.001);
  private final PIDController xSpeedController = new PIDController(0.1, 0, 0);

  public NoteHuntCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
    xSpeedController.setTolerance(2);
  }

  @Override
  public void execute() {
    if (visionSubsystem.getNoteX() == null) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double targetX = visionSubsystem.getNoteX();
    double targetY = visionSubsystem.getNoteY();

    double xSpeed = -xSpeedController.calculate(targetY, 0);
    double rot = rotSpeedController.calculate(targetX, 0);

    if (Math.abs(targetX) > 5) {
      // We want to prevent the manipulator
      // from accidently knocking the cube away.
      xSpeed = 0;
    }

    driveSubsystem.drive(xSpeed, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return (rotSpeedController.atSetpoint() && xSpeedController.atSetpoint());
  }
}

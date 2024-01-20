package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignAmpCommand extends InstantCommand {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final PIDController ySpeedController = new PIDController(0.05, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.05, 0, 0);

  public AlignAmpCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);

    ySpeedController.setTolerance(0.1);
    xSpeedController.setTolerance(0.1);
  }

  @Override
  public void execute() {
    if (!visionSubsystem.hasAmpTag()) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double angleOffTarget = visionSubsystem.getX(visionSubsystem.getAmpTagId());
    double distanceFromTarget = visionSubsystem.getDistance(visionSubsystem.getAmpTagId());

    double x = Math.cos(Math.toRadians(angleOffTarget)) * distanceFromTarget;
    double y = Math.sin(Math.toRadians(angleOffTarget)) * distanceFromTarget;

    double ySpeed = ySpeedController.calculate(x, 0);
    double xSpeed = xSpeedController.calculate(y, 0);

    driveSubsystem.drive(-xSpeed, ySpeed, 0, false);
  }

  @Override
  public boolean isFinished() {
    return ySpeedController.atSetpoint() && xSpeedController.atSetpoint();
  }
}

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.commands.DriveToPoseConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPoseCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;

  private final PIDController xSpeedController =
      new PIDController(
          DriveToPoseConstants.XSpeedPid.P,
          DriveToPoseConstants.XSpeedPid.I,
          DriveToPoseConstants.XSpeedPid.D);
  private final PIDController ySpeedController =
      new PIDController(
          DriveToPoseConstants.YSpeedPid.P,
          DriveToPoseConstants.YSpeedPid.I,
          DriveToPoseConstants.YSpeedPid.D);

  private final Pose2d targetPose;

  public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;

    xSpeedController.setTolerance(DriveToPoseConstants.XSpeedPid.TOLERANCE);
    ySpeedController.setTolerance(DriveToPoseConstants.YSpeedPid.TOLERANCE);

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSubsystem.getPose();

    double xSpeed = xSpeedController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = ySpeedController.calculate(currentPose.getY(), targetPose.getY());

    driveSubsystem.drive(xSpeed, ySpeed, 0);
  }

  @Override
  public boolean isFinished() {
    return xSpeedController.atSetpoint() && ySpeedController.atSetpoint();
  }
}

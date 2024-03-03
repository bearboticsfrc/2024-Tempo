package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final Translation2d targetPoint;

  private final PIDController xSpeedPidController = new PIDController(0, 0, 0);
  private final PIDController ySpeedPidController = new PIDController(0, 0, 0);

  /**
   * Constructs the AutoDriveCommand with the DriveSubsystem and target point.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for autonomous driving.
   */
  public AutoDriveCommand(DriveSubsystem driveSubsystem, Translation2d targetPoint) {
    this.driveSubsystem = driveSubsystem;
    this.targetPoint = targetPoint;

    xSpeedPidController.setTolerance(2);
    ySpeedPidController.setTolerance(2);

    addRequirements(driveSubsystem);
  }

  /**
   * Executes the auto-driving logic by calculating PID-controlled speed for both X and Y axes and
   * commanding the DriveSubsystem to drive accordingly.
   */
  @Override
  public void execute() {
    Translation2d currentPoint = driveSubsystem.getPose().getTranslation();

    driveSubsystem.drive(
        xSpeedPidController.calculate(currentPoint.getX(), targetPoint.getX()),
        ySpeedPidController.calculate(currentPoint.getY(), targetPoint.getY()),
        0);
  }

  /**
   * Checks if the robot has reached the setpoint for both X and Y axes.
   *
   * @return True if the robot has reached the setpoint, otherwise false.
   */
  @Override
  public boolean isFinished() {
    return xSpeedPidController.atSetpoint() && ySpeedPidController.atSetpoint();
  }
}

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private PIDController rotSpeedPidController = new PIDController(0.01, 0.01, 0.0005);

  private Translation2d targetPoint;

  private boolean aimFront = true;

  private DoubleSupplier xSupplier = () -> 0.0;
  private DoubleSupplier ySupplier = () -> 0.0;

  private DoublePublisher targetRotationPublisher;

  /*
   * Constructs the AutoAimCommand with the DriveSubsystem, target point, and optional
   * X and Y component suppliers for dynamic driving.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for auto-aiming.
   * @param xSupplier Supplier for the X component to pass to DriveSubsystem.drive.
   * @param ySupplier Supplier for the Y component to pass to DriveSubsystem.drive.
   */
  public AutoAimCommand(
      DriveSubsystem driveSubsystem,
      Translation2d targetPoint,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this(driveSubsystem, targetPoint);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  /*
   * Constructs the AutoAimCommand with the DriveSubsystem, target point, and a boolean whether the command should aim the front or back of the robot.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for auto-aiming.
   * @param aimFront Whether the command should aim the front or back of the robot.
   */
  public AutoAimCommand(
      DriveSubsystem driveSubsystem, Translation2d targetPoint, boolean aimFront) {
    this(driveSubsystem, targetPoint);
    this.aimFront = aimFront;
  }

  /**
   * Constructs the AutoAimCommand with the DriveSubsystem and target point.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for auto-aiming.
   */
  public AutoAimCommand(DriveSubsystem driveSubsystem, Translation2d targetPoint) {
    this.driveSubsystem = driveSubsystem;
    this.targetPoint = targetPoint;

    rotSpeedPidController.setTolerance(2);
    rotSpeedPidController.enableContinuousInput(-180, 180);
    rotSpeedPidController.setIZone(5.0);

    targetRotationPublisher =
        NetworkTableInstance.getDefault().getDoubleTopic("/vision/targetRotation").publish();

    addRequirements(driveSubsystem);
  }

  /**
   * Executes the auto-aiming logic by aligning the robot's heading with the specified target point.
   */
  @Override
  public void execute() {
    aimAtPoint(ySupplier, xSupplier, targetPoint);
  }

  /**
   * Aim robot at a desired point on the field
   *
   * <p>Originally from:
   * https://github.com/lasarobotics/PH2024/blob/master/src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java
   *
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param targetPoint Target point.
   */
  public void aimAtPoint(
      DoubleSupplier xRequest, DoubleSupplier yRequest, Translation2d targetPoint) {
    Rotation2d targetRotation =
        LocationHelper.getRotationToTranslation(driveSubsystem.getPose(), targetPoint);

    targetRotationPublisher.set(targetRotation.getDegrees());

    Measure<Angle> angularOffset = aimFront ? Degrees.of(180) : Degrees.of(0);

    double rotateOutput =
        rotSpeedPidController.calculate(
            driveSubsystem
                .getPose()
                .getRotation()
                .plus(Rotation2d.fromDegrees(angularOffset.in(Degrees)))
                .getDegrees(),
            targetRotation.getDegrees());

    driveSubsystem.drive(yRequest.getAsDouble(), xRequest.getAsDouble(), rotateOutput);
  }

  /** Returns true if the PID controller indicates we are aimed. */
  @Override
  public boolean isFinished() {
    return rotSpeedPidController.atSetpoint();
  }
}

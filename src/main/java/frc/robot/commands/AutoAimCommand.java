package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.commands.AutoAimConstants;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimCommand extends Command {
  private static boolean HAS_SETUP_SHUFFLEBOARD = false;

  private final DriveSubsystem driveSubsystem;
  private final PIDController rotSpeedPidController =
      new PIDController(
          AutoAimConstants.RotationPid.P,
          AutoAimConstants.RotationPid.I,
          AutoAimConstants.RotationPid.D);

  private final Supplier<Translation2d> targetPoint;
  private boolean aimFront;

  private DoubleSupplier xSupplier = () -> 0.0;
  private DoubleSupplier ySupplier = () -> 0.0;

  private Rotation2d targetRotation = new Rotation2d();

  private boolean rotationOverride;

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
      Supplier<Translation2d> targetPoint,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this(driveSubsystem, targetPoint);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  /*
   * Constructs the AutoAimCommand with the DriveSubsystem, target point, and optional
   * X and Y component suppliers for dynamic driving.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param rotationOverride The rotation to rotate to.
   */
  public AutoAimCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Rotation2d rotationOverride) {
    this(driveSubsystem, Translation2d::new);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetRotation = rotationOverride;
    this.rotationOverride = true;
  }

  /*
   * Constructs the AutoAimCommand with the DriveSubsystem, target point, and a boolean whether the command should aim the front or back of the robot.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for auto-aiming.
   * @param aimFront Whether the command should aim the front or back of the robot.
   */
  public AutoAimCommand(
      DriveSubsystem driveSubsystem, Supplier<Translation2d> targetPoint, boolean aimFront) {
    this(driveSubsystem, targetPoint);
    this.aimFront = aimFront;
  }

  /**
   * Constructs the AutoAimCommand with the DriveSubsystem and target point.
   *
   * @param driveSubsystem The DriveSubsystem instance for robot movement control.
   * @param targetPoint The target point for auto-aiming.
   */
  public AutoAimCommand(DriveSubsystem driveSubsystem, Supplier<Translation2d> targetPoint) {
    this.driveSubsystem = driveSubsystem;
    this.targetPoint = targetPoint;

    rotSpeedPidController.setTolerance(AutoAimConstants.RotationPid.TOLERANCE);
    rotSpeedPidController.enableContinuousInput(
        AutoAimConstants.RotationPid.ContinuousInput.MIN,
        AutoAimConstants.RotationPid.ContinuousInput.MAX);
    rotSpeedPidController.setIZone(AutoAimConstants.RotationPid.I_ZONE);

    if (!HAS_SETUP_SHUFFLEBOARD) {
      setupShuffleboardTab(RobotConstants.VISION_SYSTEM_TAB);
    }

    addRequirements(driveSubsystem);
  }

  private void setupShuffleboardTab(ShuffleboardTab tab) {
    HAS_SETUP_SHUFFLEBOARD = true;

    tab.addDouble("Target Rotation", () -> this.targetRotation.getDegrees());
  }

  /**
   * Executes the auto-aiming logic by aligning the robot's heading with the specified target point.
   */
  @Override
  public void execute() {
    if (rotationOverride) {
      aimAtPoint(ySupplier, xSupplier, targetRotation);
    } else {
      aimAtPoint(ySupplier, xSupplier, targetPoint.get());
    }
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
    targetRotation = LocationHelper.getRotationToTranslation(driveSubsystem.getPose(), targetPoint);
    aimAtPoint(xRequest, yRequest, targetRotation);
  }

  public void aimAtPoint(
      DoubleSupplier xRequest, DoubleSupplier yRequest, Rotation2d targetRotation) {
    this.targetRotation = targetRotation;
    Measure<Angle> angularOffset = aimFront ? Degrees.of(0) : Degrees.of(180);

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

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}

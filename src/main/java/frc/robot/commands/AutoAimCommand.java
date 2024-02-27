package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;

  public static final TrapezoidProfile.Constraints AIM_PID_CONSTRAINTS =
      new TrapezoidProfile.Constraints(720, 225);

  private final ProfiledPIDController aimPidController =
      new ProfiledPIDController(0.0025, 0, 0, AIM_PID_CONSTRAINTS, RobotConstants.CYCLE_TIME);

  private final Debouncer setpointDebouncer = new Debouncer(0.25);

  private DoubleSupplier xSupplier = () -> 0.0;
  private DoubleSupplier ySupplier = () -> 0.0;

  public AutoAimCommand(
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this(driveSubsystem, poseEstimatorSubsystem);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  public AutoAimCommand(
      DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    aimPidController.setTolerance(2.5);
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    aimAtPoint(
        ySupplier,
        xSupplier,
        poseEstimatorSubsystem::getPose,
        FieldPositions.getInstance().getSpeakerTranslation());
  }

  @Override
  public boolean isFinished() {
    return setpointDebouncer.calculate(aimPidController.atSetpoint());
  }

  /**
   * Aim robot at a desired point on the field
   *
   * <p>Originally from:
   * https://github.com/lasarobotics/PH2024/blob/master/src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java
   *
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param point Target point.
   */
  public void aimAtPoint(
      DoubleSupplier xRequest,
      DoubleSupplier yRequest,
      Supplier<Pose2d> currentPose,
      Translation2d point) {
    Pose2d pose = currentPose.get();

    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - pose.getX(), point.getY() - pose.getY());

    // Calculate necessary rotate rate
    double rotateOutput =
        aimPidController.calculate(
            pose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(),
            targetAngle.getDegrees());

    driveSubsystem.drive(xRequest.getAsDouble(), yRequest.getAsDouble(), rotateOutput);
  }
}

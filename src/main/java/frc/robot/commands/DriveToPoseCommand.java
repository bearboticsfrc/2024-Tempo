package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.StringFormatting;
import java.util.function.Supplier;

public class DriveToPoseCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;

  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);

  private final Supplier<Pose2d> poseSupplier;
  private double xSpeed;
  private Pose2d currentPose = new Pose2d();

  public DriveToPoseCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.poseSupplier = targetPoseSupplier;

    RobotConstants.VISION_SYSTEM_TAB.add("DTP - X Controller", xController);
    RobotConstants.VISION_SYSTEM_TAB.add("DTP - Y Controller", yController);
    RobotConstants.VISION_SYSTEM_TAB.addDouble(
        "PoseSupplier.getX()", () -> poseSupplier.get().getX());
    RobotConstants.VISION_SYSTEM_TAB.addDouble("xSpeed", () -> xSpeed);
    RobotConstants.VISION_SYSTEM_TAB.addString(
        "currentPose", () -> StringFormatting.poseToString(currentPose));

    xController.setTolerance(0.25);
    yController.setTolerance(0.25);

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    currentPose = driveSubsystem.getPose();

    xSpeed = xController.calculate(currentPose.getX(), poseSupplier.get().getX());
    double ySpeed = yController.calculate(currentPose.getY(), poseSupplier.get().getY());

    driveSubsystem.drive(xSpeed, ySpeed, 0);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}

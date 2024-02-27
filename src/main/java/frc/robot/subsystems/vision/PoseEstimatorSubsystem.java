package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final DriveSubsystem driveSubsystem;

  private List<VisionCamera> cameras = new ArrayList<VisionCamera>();

  private Pose2d initialPose = new Pose2d();

  private StructPublisher<Pose2d> fusedPosePublisher;
  private StructPublisher<Pose2d> drivePosePublisher;

  private List<Notifier> notifiers = new ArrayList<Notifier>();
  private List<EstimationRunnable> estimationRunnables = new ArrayList<EstimationRunnable>();

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, FieldPositions fieldPositions) {
    this.driveSubsystem = driveSubsystem;

    // Front Left
    cameras.add(
        new VisionCamera(
            "FrontLeft",
            new PhotonCamera(VisionConstants.FRONT_LEFT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA));

    // Front Right
    cameras.add(
        new VisionCamera(
            "FrontRight",
            new PhotonCamera(VisionConstants.FRONT_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA));

    // Back Right
    cameras.add(
        new VisionCamera(
            "BackRight",
            new PhotonCamera(VisionConstants.BACK_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA));

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    for (VisionCamera robotCamera : cameras) {
      EstimationRunnable estimatorRunnable =
          new EstimationRunnable(robotCamera.getNiceName(), robotCamera);
      estimationRunnables.add(estimatorRunnable);
    }

    Notifier notifier =
        new Notifier(
            () -> {
              for (EstimationRunnable estimationRunnable : estimationRunnables) {
                estimationRunnable.run();
              }
            });
    notifiers.add(notifier);

    // Start PhotonVision thread
    notifier.setName("AprilTagCameras");
    notifier.startPeriodic(RobotConstants.CYCLE_TIME);

    fusedPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("/vision/pose", Pose2d.struct).publish();

    drivePosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/vision/drivePose", Pose2d.struct)
            .publish();

    tab.addString("Pose", () -> StringFormatting.poseToString(driveSubsystem.getPose()));
  }

  public void setInitialPose(Pose2d pose) {
    this.initialPose = pose;
  }

  @Override
  public void periodic() {
    for (EstimationRunnable estimationRunnable : estimationRunnables) {
      estimatorChecker(estimationRunnable);
    }

    fusedPosePublisher.set(driveSubsystem.getPose());
  }

  public String getPoseString(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      double distance =
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));

      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }

    double poseAmbiguityFactor =
        estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                        + VisionConstants.POSE_AMBIGUITY_SHIFTER
                            * VisionConstants.POSE_AMBIGUITY_MULTIPLIER)
                    / (1
                        + ((estimation.targetsUsed.size() - 1)
                            * VisionConstants.TAG_PRESENCE_WEIGHT)));

    double confidenceMultiplier =
        Math.max(
            1,
            (Math.max(
                    1,
                    Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                        * VisionConstants.DISTANCE_WEIGHT)
                * poseAmbiguityFactor
                / (1
                    + ((estimation.targetsUsed.size() - 1)
                        * VisionConstants.TAG_PRESENCE_WEIGHT))));

    return VisionConstants.VISION_MEASUREMENT_STD_DEVS.times(confidenceMultiplier);
  }

  public void estimatorChecker(EstimationRunnable estamator) {
    EstimatedRobotPose robotPose = estamator.getLatestEstimatedPose();
    Pose2d visionPose = robotPose.estimatedPose.toPose2d();

    driveSubsystem.addVisionMeasurement(
        visionPose, robotPose.timestampSeconds, VisionConstants.VISION_STD_DEVS);
  }
}

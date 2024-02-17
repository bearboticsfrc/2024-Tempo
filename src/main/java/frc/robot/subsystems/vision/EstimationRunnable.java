package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.location.FieldPositions;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class EstimationRunnable implements Runnable {
  /** Runnable that gets AprilTag data from PhotonVision. */
  private final PhotonPoseEstimator photonPoseEstimator;

  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();

  private AprilTagFieldLayout layout;

  public EstimationRunnable(String name, AprilTagCamera camera) {
    this.photonCamera = camera.getPhotonCamera();
    PhotonPoseEstimator photonPoseEstimator = null;

    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              photonCamera,
              camera.getRobotToCameraTransform());
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    if (photonPoseEstimator == null || photonCamera == null) return;

    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    if (!photonResults.hasTargets()) return;

    if (photonResults.targets.size() == 1
        && photonResults.targets.get(0).getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD) return;

    photonPoseEstimator
        .update(photonResults)
        .ifPresent(
            estimatedRobotPose -> {
              Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
              // Make sure the measurement is on the field
              if (estimatedPose.getX() > 0.0
                  && estimatedPose.getX() <= (FieldPositions.FIELD_LENGTH + .1)
                  && estimatedPose.getY() > 0.0
                  && estimatedPose.getY() <= FieldPositions.FIELD_WIDTH) {
                atomicEstimatedRobotPose.set(estimatedRobotPose);
              }
            });
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a
   * non-null value, it is a new estimate that hasn't been returned before. This pose will be with
   * the blue alliance origin.
   *
   * @return latest estimated pose
   */
  public EstimatedRobotPose getLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }
}

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.location.FieldPositions;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class EstimationRunnable implements Runnable {
  /** Runnable that gets AprilTag data from PhotonVision. */
  private final PhotonPoseEstimator photonPoseEstimator;

  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();

  private AprilTagFieldLayout layout;

  public EstimationRunnable(String name, VisionCamera camera) {
    this.layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    this.photonCamera = camera.getPhotonCamera();

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            photonCamera,
            camera.getRobotToCameraTransform());
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void run() {
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    if (!photonResults.hasTargets()
        || (photonResults.targets.size() == 1
            && photonResults.targets.get(0).getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD)) {
      return;
    }

    if (photonResults.targets.size() == 1) {
      PhotonTrackedTarget target = photonResults.getTargets().get(0);
      Transform3d t3d = target.getBestCameraToTarget();
      double distance = Math.hypot(t3d.getX(), t3d.getY());

      if (distance > 3.5) return;
    }

    photonPoseEstimator
        .update(photonResults)
        .ifPresent(
            estimatedRobotPose -> {
              Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
              // Make sure the measurement is on the field
              if (estimatedPose.getX() > -FieldPositions.TAG_OUTSIDE_FIELD
                  && estimatedPose.getX()
                      <= (FieldPositions.FIELD_LENGTH + FieldPositions.TAG_OUTSIDE_FIELD)
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

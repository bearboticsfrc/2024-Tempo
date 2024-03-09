// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.location.LocationHelper;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;

  public ObjectDetectionSubsystem(String cameraName) {
    photonCamera = new PhotonCamera(cameraName);
  }

  public boolean hasNoteInView() {
    return photonCamera.getLatestResult().hasTargets();
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(photonCamera.getLatestResult().getBestTarget());
  }

  public Optional<Pose2d> getPoseToNearestNote(Pose2d currentPose) {
    if (!hasNoteInView()) {
      return Optional.empty();
    }

    return Optional.of(
        LocationHelper.getPoseByDistanceAndAngleToPose(
            currentPose,
            getBestTarget().get().getBestCameraToTarget().getX(),
            Rotation2d.fromDegrees(getBestTarget().get().getYaw())));
  }
}

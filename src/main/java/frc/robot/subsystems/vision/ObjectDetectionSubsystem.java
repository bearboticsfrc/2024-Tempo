// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;
  private boolean driverCameraMode;

  public ObjectDetectionSubsystem(String cameraName) {
    photonCamera = new PhotonCamera(cameraName);
  }

  public void toggleDriverCamera() {
    photonCamera.setDriverMode(driverCameraMode ^= true);
  }

  public boolean hasTargetInView() {
    return photonCamera.getLatestResult().hasTargets();
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return hasTargetInView()
        ? Optional.of(photonCamera.getLatestResult().getBestTarget())
        : Optional.empty();
  }

  public Optional<Transform3d> getTransformToBestTarget() {
    return hasTargetInView()
        ? Optional.of(getBestTarget().get().getBestCameraToTarget())
        : Optional.empty();
  }
}

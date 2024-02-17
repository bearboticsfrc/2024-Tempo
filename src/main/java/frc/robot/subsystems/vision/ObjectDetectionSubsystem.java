// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionSubsystem extends SubsystemBase {

  private PhotonCamera photonCamera = null;
  private boolean driverCameraMode = false;

  public ObjectDetectionSubsystem() {
    photonCamera = new PhotonCamera(VisionConstants.OBJECT_DETECTION_CAMERA);
  }

  public void toggleDriverCamera() {
    driverCameraMode = !driverCameraMode;
    photonCamera.setDriverMode(driverCameraMode);
  }

  public boolean hasNoteInView() {
    return photonCamera.getLatestResult().hasTargets();
  }

  public Optional<Double> getYawToNearestNote() {
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    if (!photonResults.hasTargets()) return Optional.empty();

    PhotonTrackedTarget target = photonResults.getBestTarget();

    return Optional.of(target.getYaw());
  }

  public Optional<Double> getDistanceToNearestNote() {
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    if (!photonResults.hasTargets()) return Optional.empty();

    PhotonTrackedTarget target = photonResults.getBestTarget();

    return Optional.of(target.getBestCameraToTarget().getX());
  }
}

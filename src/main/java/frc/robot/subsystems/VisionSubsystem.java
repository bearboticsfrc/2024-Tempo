// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import java.util.HashMap;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

// TODO: Update to support a multicam setup
public class VisionSubsystem extends SubsystemBase {
  private final HashMap<Integer, PhotonTrackedTarget> trackedTargets = new HashMap<>();

  private final PhotonCamera photonCamera;
  private final PhotonCamera photonCameraObject;
  private boolean driverCameraMode = false;
  private PhotonTrackedTarget note;

  public VisionSubsystem() {
    photonCamera = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
    photonCameraObject = new PhotonCamera(VisionConstants.CAMERA_2_NAME);
  }

  @Override
  public void periodic() {
    note = photonCameraObject.getLatestResult().getBestTarget();
    trackedTargets.clear();

    for (PhotonTrackedTarget target : photonCamera.getLatestResult().getTargets()) {

      trackedTargets.put(target.getFiducialId(), target);
    }
  }

  /**
   * Get the latest target by its fiducial ID.
   *
   * @param fiducialId The ID.
   * @return The target, returned as an {@link Optional} of type {@link PhotonTrackedTarget}
   */
  public Optional<PhotonTrackedTarget> getTarget(int fiducialId) {
    return Optional.ofNullable(trackedTargets.get(fiducialId));
  }

  public PhotonTrackedTarget getNote() {
    return note;
  }

  public void toggleDriverCamera() {
    driverCameraMode = !driverCameraMode;
    photonCamera.setDriverMode(driverCameraMode);
  }
}

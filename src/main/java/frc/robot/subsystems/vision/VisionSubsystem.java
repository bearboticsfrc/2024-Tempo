// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private HashMap<Integer, Double> tagX = new HashMap<>();
  private HashMap<Integer, Double> tagDistance = new HashMap<>();
  private PhotonCamera photonCamera = null;
  private boolean driverCameraMode = false;

  public VisionSubsystem() {
    photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  }

  @Override
  public void periodic() {
    tagX.clear();
    tagDistance.clear();
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();
    if (photonResults.hasTargets()) {
      List<PhotonTrackedTarget> targets = photonResults.getTargets();
      for (PhotonTrackedTarget target : targets) {
        double x = target.getYaw();
        int id = target.getFiducialId();
        double distance = target.getBestCameraToTarget().getX();
        tagX.put(id, x);
        tagDistance.put(id, distance);
      }
    }
  }

  public void toggleDriverCamera() {
    driverCameraMode = !driverCameraMode;
    photonCamera.setDriverMode(driverCameraMode);
  }

  public double getX(int tagNumber) {
    if (tagX.get(tagNumber) == null) return 0.0; // maybe this should be -1 ?
    return tagX.get(tagNumber);
  }

  public double getDistance(int tagNumber) {
    if (tagDistance.get(tagNumber) == null) return 0.0; // maybe this should be -1 ?
    return tagDistance.get(tagNumber);
  }

  public int getSpeakerCenterTag() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return VisionConstants.TAG.BLUE_SPEAKER_CENTER.getValue();
    }
    return VisionConstants.TAG.RED_SPEAKER_CENTER.getValue();
  }

  public int getAmpTagId() {
    if (AllianceColor.alliance == Alliance.Blue) {
      return VisionConstants.TAG.BLUE_AMP.getValue();
    }
    return VisionConstants.TAG.RED_AMP.getValue();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;
  private boolean driverCameraMode;
  private final DriveSubsystem drive;
  double cameraHeight = 0.5334;
  double cameraangle = 40;

  public ObjectDetectionSubsystem(String cameraName, DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    photonCamera = new PhotonCamera(cameraName);
  }

  public void toggleDriverCamera() {
    photonCamera.setDriverMode(driverCameraMode ^= true);
  }

  public boolean hasNoteInView() {
    return photonCamera.getLatestResult().hasTargets();
  }

  private PhotonTrackedTarget getBestTarget() {
    return photonCamera.getLatestResult().getBestTarget();
  }

  public Optional<PhotonTrackedTarget> getTargetToNearestNote() {
    return Optional.ofNullable(getBestTarget());
  }

  public Optional<Transform3d> getTransformToNearestNote() {
    return Optional.ofNullable(getBestTarget().getBestCameraToTarget());
  }

  public Optional<Pose2d> getPoseToNearestNote() {

    if (!hasNoteInView()) {
      return Optional.empty();
    }
    double yaw = getTargetToNearestNote().get().getYaw();
    double pitch = getTargetToNearestNote().get().getPitch() + cameraangle;
    double xDistance = cameraHeight * Math.tan(Math.toRadians(pitch));
    double yDistance = xDistance * Math.tan(Math.toRadians(yaw));
    Pose2d pose = drive.getPose();

    return Optional.of(new Pose2d(xDistance, yDistance, pose.getRotation()));
  }

  @Override
  public void periodic() {
    DataLogManager.log(getPoseToNearestNote().toString());
  }
}

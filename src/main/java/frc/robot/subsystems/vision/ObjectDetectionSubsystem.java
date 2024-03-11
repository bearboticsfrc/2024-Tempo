// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;
  private boolean driverCameraMode;
  private final DriveSubsystem drive;

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
    Pose2d currentPose = drive.getPose();

    if (!hasNoteInView()) {
      return Optional.empty();
    }

    return Optional.of(
        LocationHelper.getPoseByDistanceAndAngleToPose(
            currentPose,
            getTransformToNearestNote().get().getX(),
            Rotation2d.fromDegrees(getTargetToNearestNote().get().getYaw())));
  }

  @Override
  public void periodic() {
    DataLogManager.log(getPoseToNearestNote().toString());
  }
}

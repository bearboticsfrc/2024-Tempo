package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class AprilTagCamera {
  private String niceName;
  private PhotonCamera photonCamera;
  private Transform3d robotToCameraTransform;

  public AprilTagCamera(
      String niceName, PhotonCamera photonCamera, Transform3d robotToCameraTransform) {
    this.niceName = niceName;
    this.photonCamera = photonCamera;
    this.robotToCameraTransform = robotToCameraTransform;
  }

  public String getNiceName() {
    return niceName;
  }

  public Transform3d getRobotToCameraTransform() {
    return robotToCameraTransform;
  }

  public PhotonCamera getPhotonCamera() {
    return photonCamera;
  }
}

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.localization.CalibratedCamera;
import frc.robot.subsystems.localization.VisionCamera;
import frc.robot.util.StoreCalibratedCameras;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;

public final class CalibrateCameras {
  private static final double defaultTransform = 1;
  private static final double noNameCamera = 0;

  private static ShuffleboardTab tab = Shuffleboard.getTab("Add Cameras");
  private static GenericEntry xTab = tab.add("X: ", 1).getEntry();
  private static GenericEntry yTab = tab.add("Y: ", 1).getEntry();
  private static GenericEntry zTab = tab.add("Z: ", 1).getEntry();
  private static GenericEntry fidID = tab.add("Fiducial ID: ", 0).getEntry();
  private static GenericEntry cameraID = tab.add("camera : ", noNameCamera).getEntry();

  private static List<VisionCamera> cameras = new ArrayList<>();
  private static List<AprilTag> knownTags = FieldPositions.getInstance().getLayout().getTags();

  private static List<String> cameraNames = new ArrayList<>();

  private static List<CalibratedCamera> calibratedCameras =
      StoreCalibratedCameras.loadVersion().getCalilCalibratedDuoCameras();

  public CalibrateCameras() {
    // Front Left
    cameras.add(
        new VisionCamera(
            1,
            "FrontLeft",
            new PhotonCamera(VisionConstants.FRONT_LEFT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA));

    // Front Right
    cameras.add(
        new VisionCamera(
            2,
            "FrontRight",
            new PhotonCamera(VisionConstants.FRONT_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA));

    // Back Right
    cameras.add(
        new VisionCamera(
            3,
            "BackRight",
            new PhotonCamera(VisionConstants.BACK_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA));

    // Back Left
    cameras.add(
        new VisionCamera(
            4,
            "BackLeft",
            new PhotonCamera(VisionConstants.BACK_LEFT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA));
  }

  private static void configure() {
    VisionCamera camera = retrieveVisionCamera(cameraID.getDouble(noNameCamera));
    if (camera == null) {
      return;
    }
    int id = new Double(fidID.getDouble(0)).intValue();
    if (id == 0) {
      return;
    }
    double x = xTab.getDouble(1);
    double y = yTab.getDouble(1);
    double z = zTab.getDouble(1);
    Transform3d trans = new Transform3d(x, y, z, new Rotation3d());

    CalibratedCamera calibration = new CalibratedCamera(id, camera, trans);
    storeCalibratedCameras(calibration);
  }

  private static void storeCalibratedCameras(CalibratedCamera add) {
    for (CalibratedCamera i : calibratedCameras) {
      if ((i.getFiducialId() == add.getFiducialId()) && (i.getCamera() == add.getCamera())) {
        i = add;
        return;
      }
    }
    calibratedCameras.add(add);
  }

  private static VisionCamera retrieveVisionCamera(double id) {

    for (VisionCamera i : cameras) {
      if (id == i.getNiceNumber()) {
        return i;
      }
    }
    DataLogManager.log("no camera");
    return null;
  }
}

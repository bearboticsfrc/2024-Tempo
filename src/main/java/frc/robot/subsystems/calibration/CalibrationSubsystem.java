package frc.robot.subsystems.calibration;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.localization.CalibratedCamera;
import frc.robot.subsystems.localization.VisionCamera;
import frc.robot.util.StoreCalibratedCameras;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;

public final class CalibrationSubsystem extends SubsystemBase {
  private static final double defaultTransform = 1;
  private static final double noNameCamera = 0;
  private static final ShuffleboardTab tab = Shuffleboard.getTab("Add Cameras");

  private static final GenericEntry xTab = tab.add("X: ", 1).getEntry();
  private static final GenericEntry yTab = tab.add("Y: ", 1).getEntry();
  private static final GenericEntry zTab = tab.add("Z: ", 1).getEntry();
  private static final GenericEntry fidID = tab.add("Fiducial ID: ", 0).getEntry();

  private static GenericEntry cameraID = tab.add("camera : ", noNameCamera).getEntry();

  private static List<VisionCamera> cameras = new ArrayList<VisionCamera>();

  private static List<CalibratedCamera> calibratedCameras =
      StoreCalibratedCameras.loadVersion().getCalilCalibratedDuoCameras();

  {

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

  public static void configure() {
    VisionCamera camera =
        retrieveVisionCamera(CalibrationSubsystem.cameraID.getDouble(noNameCamera));
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
    for (CalibratedCamera i : CalibrationSubsystem.calibratedCameras) {
      if ((i.getFiducialId() == add.getFiducialId()) && (i.getCamera() == add.getCamera())) {
        i = add;
        return;
      }
    }
    CalibrationSubsystem.calibratedCameras.add(add);
  }

  private static VisionCamera retrieveVisionCamera(double id) {

    for (VisionCamera i : CalibrationSubsystem.cameras) {
      if (id == i.getNiceNumber()) {
        return i;
      }
    }
    DataLogManager.log("no camera");
    return null;
  }
}

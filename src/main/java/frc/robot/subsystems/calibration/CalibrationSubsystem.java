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

public class CalibrationSubsystem extends SubsystemBase {
  private  double defaultTransform;
  private  double noNameCamera;
  private  ShuffleboardTab tab;

  private  GenericEntry xTab;
  private GenericEntry yTab ;
  private  GenericEntry zTab ;
  private GenericEntry fidID ;

  private  GenericEntry cameraID ;

  private List<VisionCamera> cameras;

  private List<CalibratedCamera> calibratedCameras;

  public CalibrationSubsystem(){
    this.defaultTransform = 1;
    this.noNameCamera = 0;
    this.tab = Shuffleboard.getTab("Add Cameras");

    this.xTab = tab.add("X: ", 1).getEntry();
  this.yTab = tab.add("Y: ", 1).getEntry();
  this.zTab = tab.add("Z: ", 1).getEntry();
  this.fidID = tab.add("Fiducial ID: ", 0).getEntry();

  this.cameraID = tab.add("camera : ", noNameCamera).getEntry();

  this.cameras = new ArrayList<VisionCamera>();

  this.calibratedCameras =
      StoreCalibratedCameras.loadStaticVersion();

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

  public void configure() {
    VisionCamera camera =
        retrieveVisionCamera(this.cameraID.getDouble(noNameCamera));
    if (camera == null) {
      return;
    }
    int id = new Double(this.fidID.getDouble(0)).intValue();
    if (id == 0) {
      return;
    }
    double x =this. xTab.getDouble(0);
    double y = this.yTab.getDouble(0);
    double z = this.zTab.getDouble(0);
    Transform3d trans = new Transform3d(x, y, z, new Rotation3d());

    CalibratedCamera calibration = new CalibratedCamera(id, camera, trans);
    storeCalibratedCameras(calibration);
  }

  private  void storeCalibratedCameras(CalibratedCamera add) {
    if(this.calibratedCameras==null){
      return;
    }
    for (CalibratedCamera i : this.calibratedCameras) {
      if ((i.getFiducialId() == add.getFiducialId()) && (i.getCamera() == add.getCamera())) {
        i = add;
        return;
      }
    }
    this.calibratedCameras.add(add);
    return;
  }

  private VisionCamera retrieveVisionCamera(double id) {

    for (VisionCamera i : this.cameras) {
      if (id == i.getNiceNumber()) {
        return i;
      }
    }
    DataLogManager.log("no camera");
    return null;
  }
}

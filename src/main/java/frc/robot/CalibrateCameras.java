package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.localization.CalibratedDuoCameras;
import frc.robot.subsystems.localization.VisionCamera;
import frc.robot.util.StoreCalibratedCameras;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import org.photonvision.PhotonCamera;

public final class CalibrateCameras {
  private static List<VisionCamera> cameras = new ArrayList<>();
  private static List<AprilTag> knownTags = FieldPositions.getInstance().getLayout().getTags();

  private static List<String> cameraNames = new ArrayList<>();

  private static List<CalibratedDuoCameras> calibratedCameras =
      StoreCalibratedCameras.loadVersion().getCalilCalibratedDuoCameras();

  public CalibrateCameras() {
    // Front Left
    cameras.add(
        new VisionCamera(
            "FrontLeft",
            new PhotonCamera(VisionConstants.FRONT_LEFT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA));

    // Front Right
    cameras.add(
        new VisionCamera(
            "FrontRight",
            new PhotonCamera(VisionConstants.FRONT_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA));

    // Back Right
    cameras.add(
        new VisionCamera(
            "BackRight",
            new PhotonCamera(VisionConstants.BACK_RIGHT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA));

    // Back Left
    cameras.add(
        new VisionCamera(
            "BackLeft",
            new PhotonCamera(VisionConstants.BACK_LEFT_CAMERA_NAME),
            VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA));

    for (VisionCamera i : cameras) {
      cameraNames.add(i.getNiceName());
    }
  }

  public static void main(String[] args) {
    configure();
  }

  private static void configure() {
    Boolean done = false;
    while (done == false) {

      Scanner feedback = new Scanner(System.in);
      CalibratedDuoCameras response = addCalibratedDuoCameras(feedback);
      if (response == null) {
        return;
      } else {
        storeCalibratedCameras(response);
      }
    }
  }

  private static void storeCalibratedCameras(CalibratedDuoCameras add) {
    for (CalibratedDuoCameras i : calibratedCameras) {
      if ((i.getFiducialId() == add.getFiducialId())
          && ((i.getCameras()[0] == add.getCameras()[0])
              || (i.getCameras()[0] == add.getCameras()[1]))
          && ((i.getCameras()[1] == add.getCameras()[0])
              || (i.getCameras()[1] == add.getCameras()[1]))) {
        i = add;
        return;
      }
    }
    calibratedCameras.add(add);
  }

  private static VisionCamera retrieveVisionCamera(String name) {

    for (VisionCamera i : cameras) {
      if (name == i.getNiceName()) {
        return i;
      }
    }
    return new VisionCamera(null, null, null);
  }

  private static CalibratedDuoCameras addCalibratedDuoCameras(Scanner feedback) {
    System.out.println("camera one: ");
    VisionCamera cameraOne = addCamera(feedback);
    if (cameraOne == null) {
      return null;
    }
    System.out.println("camera Two: ");
    VisionCamera cameraTwo = addCamera(feedback);
    if (cameraTwo == null) {
      return null;
    }

    System.out.println("fiducial id: ");
    int fidID = Integer.valueOf(feedback.nextLine());
    AprilTag tagToCalb = tagFromID(fidID);
    if (tagToCalb != null) {
      System.out.println("tag to bot transform x: ");
      double x = Double.valueOf(feedback.nextLine());
      System.out.println("tag to bot transform y: ");
      double y = Double.valueOf(feedback.nextLine());
      System.out.println("tag to bot transform z: ");
      double z = Double.valueOf(feedback.nextLine());
      System.out.println("tag to bot transform rot: ");
      double rot = Double.valueOf(feedback.nextLine());

      return new CalibratedDuoCameras(
          tagToCalb, cameraOne, cameraTwo, new Transform3d(x, y, z, new Rotation3d(rot, 0, 0)));
    }
    System.out.println("fid id not in field layout");
    return null;
  }

  private static AprilTag tagFromID(int id) {
    for (AprilTag i : knownTags) {
      if (i.ID == id) {
        return i;
      }
    }
    return null;
  }

  private static VisionCamera addCamera(Scanner feedback) {

    System.out.println("camera name: ");
    String input = feedback.nextLine().toUpperCase();
    if (cameraNames.contains(input)) {
      System.out.println("added " + input);
      return retrieveVisionCamera(input);

    } else {
      System.out.println(input + " not found");
      System.out.println("add camera Y/N: ");
      input = feedback.nextLine().toUpperCase();
      if (input == "Y") {
        return addCamera(feedback);
      } else {
        return null;
      }
    }
  }
}

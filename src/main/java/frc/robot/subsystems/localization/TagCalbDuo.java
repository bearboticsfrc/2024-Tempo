package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.vision.VisionCamera;
import java.util.*;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// purely for the sake of simplicity the Z vector quantity
// is used for algebraic representation only
// this quantity will be kicked by a robot constant at the end of the program
// the main call back of the tag calb duo class is the Transform3d function
// CalibratedThreeDimensionalVector
// kickers must use degrees
// tag calb will use two average valued camera to tag transform vectors from two different cameras
// on a single tag
// these two cameras will be positioned on two ends of a single side of the robot prefferable aiming
// towards each other(inverted)
// the angle of intersection should ideally be perpendicular
// if the two cameras cannot be positioned towards each other in this manner due to the priority of
// some other robot design constraint
// then such cameras should ideally have their focus aiming towards the tag
public class TagCalbDuo {

  private final int calbIterations = 10;

  private AprilTag tagToCalb;

  private double kickerZOne;
  private double kickerXOne;

  private double kickerZTwo;
  private double kickerXTwo;

  private VisionCamera localizingCameraOne;
  private VisionCamera localizingCameraTwo;

  private Transform3d cameraToTagTransformOne;
  private Transform3d cameraToTagTransformTwo;

  double knownCosinudalVector;

  DoubleSupplier localizingCameraOneSupplierZ =
      () ->
          naturalCameraToTag(localizingCameraOne.getPhotonCamera().getLatestResult(), tagToCalb.ID)
              .getZ();

  DoubleSupplier localizingCameraOneSupplierX =
      () ->
          naturalCameraToTag(localizingCameraOne.getPhotonCamera().getLatestResult(), tagToCalb.ID)
              .getX();

  DoubleSupplier localizingCameraTwoSupplierZ =
      () ->
          naturalCameraToTag(localizingCameraTwo.getPhotonCamera().getLatestResult(), tagToCalb.ID)
              .getZ();

  DoubleSupplier localizingCameraTwoSupplierX =
      () ->
          naturalCameraToTag(localizingCameraOne.getPhotonCamera().getLatestResult(), tagToCalb.ID)
              .getX();

  public TagCalbDuo(
      boolean inverted,
      AprilTag tagToCalb,
      VisionCamera localizingCameraOne,
      VisionCamera localizingCameraTwo,
      Transform3d knownRobotToTagTransform) {
    this.tagToCalb = tagToCalb;
    this.localizingCameraOne = localizingCameraOne;
    this.localizingCameraTwo = localizingCameraTwo;

    List<Transform3d> oneToTagList = new ArrayList<Transform3d>();
    oneToTagList.add(knownRobotToTagTransform);
    oneToTagList.add(localizingCameraOne.getRobotToCameraTransform());
    this.cameraToTagTransformOne = addThreeDimensionalVectors(oneToTagList);

    List<Transform3d> twoToTagList = new ArrayList<Transform3d>();
    oneToTagList.add(knownRobotToTagTransform);
    oneToTagList.add(localizingCameraTwo.getRobotToCameraTransform());
    this.cameraToTagTransformTwo = addThreeDimensionalVectors(twoToTagList);

    kickerZOne =
        sinusoidalKicker(
            localizingCameraOneSupplierZ,
            knownRobotToTagTransform.getZ(),
            knownRobotToTagTransform.getY());

    kickerXOne =
        sinusoidalKicker(
            localizingCameraOneSupplierX,
            knownRobotToTagTransform.getY(),
            knownRobotToTagTransform.getX());

    kickerZTwo =
        sinusoidalKicker(
            localizingCameraTwoSupplierZ,
            knownRobotToTagTransform.getZ(),
            knownRobotToTagTransform.getY());

    kickerXTwo =
        sinusoidalKicker(
            localizingCameraTwoSupplierX,
            knownRobotToTagTransform.getY(),
            knownRobotToTagTransform.getX());
  }

  public Transform3d CalibratedThreeDimensionalVector() {

    PhotonPipelineResult resultOne = localizingCameraOne.getPhotonCamera().getLatestResult();

    PhotonPipelineResult resultTwo = localizingCameraTwo.getPhotonCamera().getLatestResult();

    double qZOne = 0;
    double qZTwo = 0;
    double qYOne = 0;
    double qYTwo = 0;
    double qXOne = 0;
    double qXTwo = 0;

    // appply angular quantities to y

    return new Transform3d();
  }

  private Transform3d naturalCameraToTag(PhotonPipelineResult latestResult, int tag) {

    PhotonTrackedTarget target = findTarget(latestResult.getTargets(), tag);
    if (target == null) {
      return new Transform3d();
    } else {
      return target.getBestCameraToTarget();
    }
  }

  private PhotonTrackedTarget findTarget(List<PhotonTrackedTarget> targets, int fiducialId) {
    if (targets.isEmpty()) {
      return null;
    }
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == fiducialId) {
        return target;
      }
    }
    return null;
  }

  private double sinusoidalKicker(
      DoubleSupplier oneDimensionalSinudalVectorSupplier,
      double knownSinudalVector,
      double knownCosinudalVector) {

    double hypotenuse = Math.hypot(knownSinudalVector, knownCosinudalVector);
    double k = 0;

    for (int i = 0; i < calbIterations; ) {
      double sinusoidalVector = oneDimensionalSinudalVectorSupplier.getAsDouble();
      if (sinusoidalVector != 0) {
        k += sinusoidalVector;
        i++;
      } else {
        DataLogManager.log("calibration tag not found");
      }
    }
    k /= calbIterations;
    return Math.asin(knownSinudalVector / hypotenuse) - k;
  }

  public Transform3d addThreeDimensionalVectors(List<Transform3d> vectors) {
    Transform3d summnationVector = new Transform3d();
    for (Transform3d vector : vectors) {
      summnationVector =
          new Transform3d(
              summnationVector.getX() + vector.getX(),
              summnationVector.getY() + vector.getY(),
              summnationVector.getZ() + vector.getZ(),
              new Rotation3d());
    }
    return summnationVector;
  }

  // this has a stationless rotation quantity
  // because pure linear algebra should not have a rotation quantity on a vector
  // because rotation has no effect on the mathmatical quantity of a vector anyway
  // vectors are not a coordinate system they are quanties
  // a geometric point 3d space should be represented seperately by a 4 dimensional set
  // consisting of 3 different vector quantities
  // and a 3d vector defined in a field for rotation
  // there fore these quantities should be placed in a seperate object
  // therefore the rotation quantity Vector from this function is fundamentally useless

}

package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.vision.VisionCamera;
import java.util.*;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// there are no rotational vector quantities for sake of mathmatical sophistication
// these quantities are in a seperate field
// and it doesn't make sense to define this space as as set
// and it allows for a intuitive geometric representation of this transform
// purely for the sake of simplicity the Z vector quantity
// is used for algebraic representation only
// CalibratedThreeDimensionalVector
// unit are in radians
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

  private double kickerYOne;
  private double kickerXOne;

  private double kickerYTwo;
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

    kickerYOne =
        sinusoidalKicker(
            localizingCameraOneSupplierZ,
            cameraToTagTransformOne.getZ(),
            cameraToTagTransformOne.getY());

    kickerXOne =
        sinusoidalKicker(
            localizingCameraOneSupplierX,
            cameraToTagTransformOne.getY(),
            cameraToTagTransformOne.getX());

    kickerYTwo =
        sinusoidalKicker(
            localizingCameraTwoSupplierZ,
            cameraToTagTransformTwo.getZ(),
            cameraToTagTransformTwo.getY());

    kickerXTwo =
        sinusoidalKicker(
            localizingCameraTwoSupplierX,
            cameraToTagTransformTwo.getY(),
            cameraToTagTransformTwo.getX());
  }

  public Transform3d CalibratedThreeDimensionalVector() {

    PhotonPipelineResult resultOne = localizingCameraOne.getPhotonCamera().getLatestResult();

    PhotonPipelineResult resultTwo = localizingCameraTwo.getPhotonCamera().getLatestResult();

    Transform3d naturalCameraToTagOne = naturalCameraToTag(resultOne, this.tagToCalb.ID);
    Transform3d naturalCameraToTagTwo = naturalCameraToTag(resultTwo, this.tagToCalb.ID);

    if ((naturalCameraToTagOne == null) || (naturalCameraToTagTwo == null)) {
      return null;
    }

    // since quantity z is given

    double qZ = FieldPositions.getInstance().getTagPose3d(this.tagToCalb.ID).getZ();

    double estimatedQYOne = naturalCameraToTagOne.getY();
    double estimatedQYTwo = naturalCameraToTagTwo.getY();

    double qYOne = applyAngularKickers(qZ, estimatedQYOne, kickerYOne);
    double qYTwo = applyAngularKickers(qZ, estimatedQYTwo, kickerYTwo);

    double estimatedQXOne = naturalCameraToTagOne.getX();
    double estimatedQXTwo = naturalCameraToTagTwo.getX();

    double qXOne = applyAngularKickers(qYOne, estimatedQXOne, kickerXOne);
    double qXTwo = applyAngularKickers(qYTwo, estimatedQXTwo, kickerXTwo);

    //

    return new Transform3d();
  }

  private double applyAngularKickers(
      double knownSinudal, double estimatedCosinudal, double sinudalAngularKicker) {

    return knownSinudal
        / (Math.sin(Math.atan(knownSinudal / estimatedCosinudal) + sinudalAngularKicker));
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
}

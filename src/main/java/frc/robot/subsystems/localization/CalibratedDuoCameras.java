package frc.robot.subsystems.localization;

import static frc.robot.constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.location.FieldPositions;
import java.util.*;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// units radians
// applys trigonometric kickers to produce a more accurate transform
// from a calibrated 3d vector space

public class CalibratedDuoCameras {

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

  public CalibratedDuoCameras(
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
      if ((naturalCameraToTagOne == null)) {
        DataLogManager.log("camera one not found");
      }
      if ((naturalCameraToTagTwo == null)) {
        DataLogManager.log("camera two not found");
      }
      return null;
    }

    // since quantity z is given

    double qZ = FieldPositions.getInstance().getTagPose3d(this.tagToCalb.ID).getZ();

    double estimatedQYOne = naturalCameraToTagOne.getY();
    double estimatedQYTwo = naturalCameraToTagTwo.getY();

    // kick y

    double qYOne = applyAngularKickers(qZ, estimatedQYOne, kickerYOne);
    double qYTwo = applyAngularKickers(qZ, estimatedQYTwo, kickerYTwo);

    double estimatedQXOne = naturalCameraToTagOne.getX();
    double estimatedQXTwo = naturalCameraToTagTwo.getX();

    // kick x

    double qXOne = applyAngularKickers(qYOne, estimatedQXOne, kickerXOne);
    double qXTwo = applyAngularKickers(qYTwo, estimatedQXTwo, kickerXTwo);

    Translation3d calibratedTranslationOne = new Translation3d(qXOne, qYOne, qZ);
    Translation3d calibratedTranslationTwo = new Translation3d(qXTwo, qYTwo, qZ);

    Transform3d calibratedTransform3dOne =
        new Transform3d(calibratedTranslationOne, naturalCameraToTagOne.getRotation());

    Transform3d calibratedTransform3dTwo =
        new Transform3d(calibratedTranslationTwo, naturalCameraToTagTwo.getRotation());

    // apply robot Transform

    List<Transform3d> calibratedRobotOne = new ArrayList<Transform3d>();
    calibratedRobotOne.add(calibratedTransform3dOne);
    calibratedRobotOne.add(localizingCameraOne.getRobotToCameraTransform().times(-1));
    Transform3d calibratedPoseOne = addThreeDimensionalVectors(calibratedRobotOne);

    List<Transform3d> calibratedRobotTwo = new ArrayList<Transform3d>();
    calibratedRobotTwo.add(calibratedTransform3dTwo);
    calibratedRobotTwo.add(localizingCameraTwo.getRobotToCameraTransform().times(-1));
    Transform3d calibratedPoseTwo = addThreeDimensionalVectors(calibratedRobotTwo);

    List<Transform3d> calibratedPoses = new ArrayList<Transform3d>();
    calibratedPoses.add(calibratedPoseOne);
    calibratedPoses.add(calibratedPoseTwo);

    return addThreeDimensionalVectors(calibratedPoses);
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
      if (target.getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD) {
        return null;
      } else if (target.getFiducialId() == fiducialId) {
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
      List<Rotation3d> rotationSum = new ArrayList<Rotation3d>();
      rotationSum.add(summnationVector.getRotation());
      rotationSum.add(vector.getRotation());
      summnationVector =
          new Transform3d(
              summnationVector.getX() + vector.getX(),
              summnationVector.getY() + vector.getY(),
              summnationVector.getZ() + vector.getZ(),
              addRotation3d(rotationSum));
    }
    return summnationVector;
  }

  public static Rotation3d addRotation3d(List<Rotation3d> rotations) {
    Rotation3d sumnationRotation = new Rotation3d();
    for (Rotation3d rotation : rotations) {
      sumnationRotation =
          new Rotation3d(
              rotation.getX() + sumnationRotation.getX(),
              rotation.getY() + sumnationRotation.getY(),
              rotation.getZ() + sumnationRotation.getZ());
    }
    return sumnationRotation;
  }

  public int getFiducialId() {
    return tagToCalb.ID;
  }

  public VisionCamera[] getCameras() {
    return new VisionCamera[] {localizingCameraOne, localizingCameraTwo};
  }
}

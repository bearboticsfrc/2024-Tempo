package frc.robot.subsystems.calibration;

import static frc.robot.constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.localization.VisionCamera;
import java.util.*;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// units radians
// applys trigonometric kickers to produce a more accurate transform
// from a calibrated 3d vector space

public class CalibratedCamera extends SubsystemBase {

  private final int calbIterations = 10;

  private int tagID;

  private double kickerY;
  private double kickerX;

  private VisionCamera localizingCamera;

  private Transform3d cameraToTagTransform;

  double knownCosinudalVector;

  DoubleSupplier localizingCameraSupplierZ =
      () -> naturalCameraToTag(localizingCamera.getPhotonCamera().getLatestResult(), tagID).getZ();

  DoubleSupplier localizingCameraSupplierX =
      () -> naturalCameraToTag(localizingCamera.getPhotonCamera().getLatestResult(), tagID).getX();

  public CalibratedCamera(
      int tagID, VisionCamera localizingCameraOne, Transform3d knownRobotToTagTransform)
      throws InterruptedException {

    this.tagID = tagID;
    this.localizingCamera = localizingCameraOne;

    List<Transform3d> ToTagList = new ArrayList<Transform3d>();
    ToTagList.add(knownRobotToTagTransform);
    ToTagList.add(localizingCameraOne.getRobotToCameraTransform());
    this.cameraToTagTransform = addThreeDimensionalVectors(ToTagList);

    kickerY =
        sinusoidalKicker(
            localizingCameraSupplierZ, cameraToTagTransform.getZ(), cameraToTagTransform.getY());

    kickerX =
        sinusoidalKicker(
            localizingCameraSupplierX, cameraToTagTransform.getY(), cameraToTagTransform.getX());
  }

  public Transform3d CalibratedThreeDimensionalVector() {

    PhotonPipelineResult result = localizingCamera.getPhotonCamera().getLatestResult();

    Transform3d naturalCameraToTag = naturalCameraToTag(result, this.tagID);

    if ((naturalCameraToTag == null)) {

      DataLogManager.log("camera not found");

      return null;
    }

    // since quantity z is given

    double qZ = FieldPositions.getInstance().getTagPose3d(this.tagID).getZ();

    double estimatedQY = naturalCameraToTag.getY();

    // kick y

    double qY = applyAngularKickers(qZ, estimatedQY, kickerY);

    double estimatedQX = naturalCameraToTag.getX();

    // kick x

    double qX = applyAngularKickers(qY, estimatedQX, kickerX);

    Translation3d calibratedTranslation = new Translation3d(qX, qY, qZ);

    Transform3d calibratedTransform3d =
        new Transform3d(calibratedTranslation, naturalCameraToTag.getRotation());

    // apply robot Transform

    List<Transform3d> calibratedRobot = new ArrayList<Transform3d>();
    calibratedRobot.add(calibratedTransform3d);
    calibratedRobot.add(localizingCamera.getRobotToCameraTransform().times(-1));
    Transform3d calibratedPose = addThreeDimensionalVectors(calibratedRobot);

    return calibratedPose;
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
      double knownCosinudalVector)
      throws InterruptedException {

    double hypotenuse = Math.hypot(knownSinudalVector, knownCosinudalVector);
    double k = 0;

    for (int i = 0; i < calbIterations; ) {
      DataLogManager.log("starting calibration");
      Double sinusoidalVector = oneDimensionalSinudalVectorSupplier.getAsDouble();
      if ((sinusoidalVector != 0) && (sinusoidalVector != null)) {
        k += sinusoidalVector;
        i++;
        DataLogManager.log("new iteration");
      } else {
        DataLogManager.log("calibration tag not found");
      }
      //Thread.sleep(Double.valueOf(RobotConstants.CYCLE_TIME).longValue());
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
    return this.tagID;
  }

  public VisionCamera getCamera() {
    return localizingCamera;
  }
}

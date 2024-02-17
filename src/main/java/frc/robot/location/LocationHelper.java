package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LocationHelper {

  public static Rotation2d getDirection(Transform2d transform) {
    return getDirection(transform.getTranslation());
  }

  public static Rotation2d getDirection(Translation2d transform) {
    return new Rotation2d(transform.getX(), transform.getY());
  }

  public static double getDistance(Transform2d transform) {
    return getDistance(transform.getTranslation());
  }

  public static double getDistance(Translation2d transform) {
    return transform.getNorm();
  }

  public static Pose2d getPoseByDistanceAndAngleToPose(
      Pose2d pose, double distance, Rotation2d angle) {
    return pose.transformBy(
        new Transform2d(new Translation2d(distance, angle).unaryMinus(), angle));
  }

  public static Translation2d getFieldRelativeLinearSpeedsMPS(DriveSubsystem driveSubsystem) {
    ChassisSpeeds robotRelativeSpeeds =
        RobotConstants.DRIVE_KINEMATICS.toChassisSpeeds(driveSubsystem.getModuleStates());

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            driveSubsystem.getPose().getRotation().unaryMinus());

    Translation2d translation =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

    if (getDistance(translation) < 0.01) {
      return new Translation2d();
    } else {
      return translation;
    }
  }

  public static double getDistanceToPose(Pose2d fromPose, Pose2d toPose) {
    return Math.abs((fromPose.getX() - toPose.getX()) + (fromPose.getY() - toPose.getY()));
  }

  public static Transform3d normalizeCameraAngle(Transform3d cameraToTarget) {
    double angle = Math.atan(cameraToTarget.getZ() / cameraToTarget.getX());
    double theta = -angle + Units.degreesToRadians(20);
    double hyp =
        Math.sqrt(
            (cameraToTarget.getX() * cameraToTarget.getX())
                + (cameraToTarget.getZ() * cameraToTarget.getZ()));
    double xPrime = hyp * Math.cos(theta);
    double zPrime = -hyp * Math.sin(theta);

    return new Transform3d(
        new Translation3d(xPrime, cameraToTarget.getY(), zPrime), cameraToTarget.getRotation());
  }

  public static void main(String[] args) {
    Pose2d pose = new Pose2d(10.0, 5.0, Rotation2d.fromDegrees(45.0));
    Pose2d newPose =
        LocationHelper.getPoseByDistanceAndAngleToPose(pose, 3.0, Rotation2d.fromDegrees(0.0));

    System.out.println("pose = " + pose);
    System.out.println("new pose = " + newPose);
  }
}

package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.calibration.CalibratedCamera;
import frc.robot.util.StoreCalibratedCameras;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final DriveSubsystem driveSubsystem;
  private final List<CalibratedCamera> cameras;
  private Pose2d pose;

  private List<Notifier> notifiers = new ArrayList<>();

  private StructPublisher<Pose2d> fusedPosePublisher;
  private DoublePublisher headingPublisher;

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, FieldPositions fieldPositions) {
    pose = new Pose2d();

    this.driveSubsystem = driveSubsystem;
    cameras = StoreCalibratedCameras.loadStaticVersion();

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    Notifier notifier = new Notifier(() -> estimator());
    notifiers.add(notifier);

    // Start PhotonVision thread
    notifier.setName("AprilTagCameras");
    notifier.startPeriodic(RobotConstants.CYCLE_TIME);

    fusedPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("/vision/pose", Pose2d.struct).publish();

    headingPublisher =
        NetworkTableInstance.getDefault().getDoubleTopic("/vision/heading").publish();

    tab.addString("Pose", () -> StringFormatting.poseToString(getPose2d()));
  }

  public void estimator() {
    int count = 0;
    Pose3d sumnationPose = new Pose3d();
    if (cameras == null) {
      return;
    }
    for (CalibratedCamera duo : cameras) {
      Transform3d duoTransform = duo.CalibratedThreeDimensionalVector();
      if (duoTransform != null) {
        Pose3d tagPose = FieldPositions.getInstance().getTagPose3d(duo.getFiducialId());
        Pose3d duoPose = tagPose.plus(duoTransform);
        sumnationPose = addPoses(sumnationPose, duoPose);
        count += 1;
      }
    }
    if (count == 0) {
      return;
    }
    sumnationPose.div(count);

    driveSubsystem.addCalibratedVisionPose(sumnationPose);
  }

  public Pose2d getPose2d() {
    return this.pose;
  }

  public Pose3d addPoses(Pose3d one, Pose3d two) {
    double x = one.getX() + two.getX();
    double y = one.getY() + two.getY();
    double z = one.getZ() + two.getZ();
    List<Rotation3d> rotList = new ArrayList<Rotation3d>();
    rotList.add(one.getRotation());
    rotList.add(two.getRotation());

    Rotation3d rot = CalibratedCamera.addRotation3d(rotList);
    return new Pose3d(x, y, z, rot);
  }

  @Override
  public void periodic() {
    Optional<Pose2d> posePer = driveSubsystem.getCalibratedVisionPose();

    if (posePer.isPresent()) {
      pose = posePer.get();

      headingPublisher.set(
          posePer.get().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees());

      fusedPosePublisher.set(posePer.get());
    }
  }
}

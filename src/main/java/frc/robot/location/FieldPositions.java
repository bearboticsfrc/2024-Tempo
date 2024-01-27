package frc.robot.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.fms.AllianceReadyListener;
import java.io.IOException;
import java.util.Optional;

public class FieldPositions implements AllianceReadyListener {

  public static double FIELD_LENGTH = 16.451; // meters
  public static double FIELD_WIDTH = 8.211; // meters

  private AprilTagFieldLayout layout;

  private static FieldPositions instance = null;

  public FieldPositions() {
    this(AllianceColor.alliance == Alliance.Red);
  }

  public FieldPositions(boolean redOrigin) {
    initializeLayout(redOrigin);
    AllianceColor.addListener(this);
  }

  public static FieldPositions getInstance() {
    if (instance == null) {
      instance = new FieldPositions();
    }
    return instance;
  }

  private void initializeLayout(boolean redOrigin) {
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
  }

  public AprilTagFieldLayout getLayout() {
    return layout;
  }

  public void updateAllianceColor(Alliance alliance) {
    initializeLayout(AllianceColor.alliance == Alliance.Red);
  }

  public double getWingY() {
    double wingY = 5.87;
    if (AllianceColor.alliance == Alliance.Red) {
      wingY = FIELD_LENGTH - 5.87;
    }
    return wingY;
  }

  public Pose3d getW1() {
    double y = (FIELD_WIDTH / 2.0) + (1.45 * 2.0);
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getW2() {
    double y = FIELD_WIDTH / 2.0 + 1.45;
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getW3() {
    double y = FIELD_WIDTH / 2.0;
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getC1() {
    double y = FIELD_WIDTH / 2.0 + (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getC2() {
    double y = FIELD_WIDTH / 2.0 + 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getC3() {
    double y = FIELD_WIDTH / 2.0;
    double x = FIELD_LENGTH / 2.0;
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getC4() {
    double y = FIELD_WIDTH / 2.0 - 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public Pose3d getC5() {
    double y = FIELD_WIDTH / 2.0 - (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  public int getSpeakerCenterTagID() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_CENTER.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SPEAKER_CENTER.getValue();
    }
    return tagId;
  }

  public Pose3d getSpeakerCenter() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_CENTER.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SPEAKER_CENTER.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getSpeakerOffset() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SPEAKER_RIGHT.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public int getAmpTagId() {
    int tagId = VisionConstants.TAG.BLUE_AMP.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_AMP.getValue();
    }
    return tagId;
  }

  public Pose3d getAmp() {
    int tagId = VisionConstants.TAG.BLUE_AMP.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_AMP.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getSourceLeft() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SOURCE_LEFT.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getSourceRight() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_RIGHT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SOURCE_RIGHT.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getStageLeft() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_STAGE_LEFT.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getStageCenter() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_CENTER.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_STAGE_CENTER.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Pose3d getStageRight() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();
    }
    return layout.getTagPose(tagId).get();
  }

  public Optional<Pose3d> getTagPose(int tag) {
    return layout.getTagPose(tag);
  }
}

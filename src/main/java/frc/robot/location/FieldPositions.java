package frc.robot.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.fms.AllianceReadyListener;
import frc.robot.constants.VisionConstants;
import java.io.IOException;

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

  public Pose2d getW1() {
    double y = (FIELD_WIDTH / 2.0) + (1.45 * 2.0);
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getW2() {
    double y = FIELD_WIDTH / 2.0 + 1.45;
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getW3() {
    double y = FIELD_WIDTH / 2.0;
    double x = 2.90;
    if (AllianceColor.alliance == Alliance.Red) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC1() {
    double y = FIELD_WIDTH / 2.0 + (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC2() {
    double y = FIELD_WIDTH / 2.0 + 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC3() {
    double y = FIELD_WIDTH / 2.0;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC4() {
    double y = FIELD_WIDTH / 2.0 - 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC5() {
    double y = FIELD_WIDTH / 2.0 - (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getSpeakerCenter() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_CENTER.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SPEAKER_CENTER.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getSpeakerOffset() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SPEAKER_RIGHT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getAmp() {
    int tagId = VisionConstants.TAG.BLUE_AMP.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_AMP.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getSourceLeft() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SOURCE_LEFT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getSourceRight() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_RIGHT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_SOURCE_RIGHT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageLeft() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_LEFT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_STAGE_LEFT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageCenter() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_CENTER.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.RED_STAGE_CENTER.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageRight() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();

    if (AllianceColor.alliance == Alliance.Red) {
      tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getTagPose(int tag) {
    return layout.getTagPose(tag).get().toPose2d();
  }
}

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

  public static final String CAMERA_1_NAME = "Arducam_OV9281_USB_Camera"; // "OV9281_2";
  public static final String OBJECT_CAMERA_NAME = "Arducam_OV9782_USB_Camera"; // "OV9281_2";

  // old settings before using camera_to_robot then inverse.
  public static final Transform3d ROBOT_TO_RIGHT_CAMERA =
      new Transform3d(
          // camera is .0717296 meters back, -0.2270252 left, 1.2596368 up
          // new Translation3d(.0717296, -0.2270252, 1.2596368),
          new Translation3d(.035, -0.2270252, 1.2596368),
          new Rotation3d(Math.toRadians(-7.0), Math.toRadians(25.0), Math.toRadians(-20.0)));
  public static final Transform3d ROBOT_TO_LEFT_CAMERA =
      new Transform3d(
          // camera is .0717296 meters back, 0.2270252 left, 1.2596368 up
          // new Translation3d(.0717296, 0.2270252, 1.2596368),
          new Translation3d(.035, 0.2270252, 1.2596368),
          new Rotation3d(Math.toRadians(7.0), Math.toRadians(25.0), Math.toRadians(20.0)));

  public static final Transform3d MIDDLE_CAMERA_TO_ROBOT =
      new Transform3d(
          new Translation3d(-0.0717296, 0.0, -1.2596368),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-20.0), Math.toRadians(0.0)));
  public static final Transform3d ROBOT_TO_MIDDLE_CAMERA = MIDDLE_CAMERA_TO_ROBOT.inverse();

  public static final Transform3d RIGHT_CAMERA_TO_ROBOT =
      new Transform3d(
          // camera is .0717296 meters back, -0.2270252 left, 1.2596368 up
          new Translation3d(.0717296, 0.2270252, -1.2596368),
          new Rotation3d(Math.toRadians(7.0), Math.toRadians(25.0), Math.toRadians(20.0)));

  //  public static final Transform3d ROBOT_TO_RIGHT_CAMERA = RIGHT_CAMERA_TO_ROBOT.inverse();

  public static final Transform3d LEFT_CAMERA_TO_ROBOT =
      new Transform3d(
          // camera is .0717296 meters back, 0.2270252 left, 1.2596368 up
          new Translation3d(.0717296, -0.2270252, -1.2596368),
          new Rotation3d(Math.toRadians(-7.0), Math.toRadians(25.0), Math.toRadians(-20.0)));

  //  public static final Transform3d ROBOT_TO_LEFT_CAMERA = LEFT_CAMERA_TO_ROBOT.inverse();

  public static final Vector<N3> STATE_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
  public static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(60));

  public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
      Matrix.mat(Nat.N3(), Nat.N1()).fill(1, 1, 1 * Math.PI);

  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double NOISY_DISTANCE_METERS = 2.5;
  public static final double DISTANCE_WEIGHT = 7;

  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  public static final double HALF_ROBOT_LENGTH = .44;

  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera"; // "OV9281_2";

  public enum TAG {
    BLUE_SOURCE_RIGHT(1),
    BLUE_SOURCE_LEFT(2),
    RED_SPEAKER_RIGHT(3),
    RED_SPEAKER_CENTER(10),
    RED_AMP(5),
    BLUE_AMP(6),
    BLUE_SPEAKER_CENTER(7),
    BLUE_SPEAKER_LEFT(8),
    RED_SOURCE_RIGHT(9),
    RED_SOURCE_LEFT(10),
    RED_STAGE_LEFT(11),
    RED_STAGE_RIGHT(12),
    RED_STAGE_CENTER(13),
    BLUE_STAGE_CENTER(14),
    BLUE_STAGE_LEFT(15),
    BLUE_STAGE_RIGHT(16);

    public final int tagNumber;

    public int getValue() {
      return tagNumber;
    }

    private TAG(int tagNumber) {
      this.tagNumber = tagNumber;
    }
  }
}

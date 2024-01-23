package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotConstants {
  public static final double TRACK_WIDTH = 000;
  public static final double WHEEL_BASE = 000;
  public static final double WHEEL_DIAMETER = 000;
  public static final int PIGEON_CAN_ID = 000;

  public static final double ROBOT_LOOP_HERTZ = 50;
  public static final double ROBOT_LOOP_PERIOD = 1 / ROBOT_LOOP_HERTZ;

  public static final ShuffleboardTab MANIPULATOR_SYSTEM_TAB =
      Shuffleboard.getTab("Manipulator System");

  public static final Translation2d FRONT_LEFT = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
  public static final Translation2d FRONT_RIGHT =
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
  public static final Translation2d BACK_LEFT = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
  public static final Translation2d BACK_RIGHT =
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

  public static final Translation2d[] moduleLocations = {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  };
  // Order here must line-up with SwerveCorner order!!
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

  public static double ROBOT_RADIUS = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2).getNorm();

  public enum SwerveCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }
}

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotConstants {
  public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
  public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
  public static final double WHEEL_DIAMETER = 0.0962; // TODO: Measure again
  public static final int PIGEON_CAN_ID = 20;

  public static final double CYCLE_TIME = Units.millisecondsToSeconds(20);

  // Order here must line-up with SwerveCorner order!!
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FL
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FR
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // BL
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // BR

  public enum SwerveCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }
}

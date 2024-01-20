package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.0;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
  public static final PIDController X_SPEED_CONTROLLER = new PIDController(0.1, 0, 0);
  public static final PIDController Y_SPEED_CONTROLLER = new PIDController(0.1, 0, 0);
  public static final PIDController THETA_SPEED_CONTROLLER = new PIDController(0.01, 0, 0);

  public static final double P_TURN_DRIVE_TO_AMP = 0.01;
  public static final double I_TURN_DRIVE_TO_AMP = 0.001;
  public static final double D_TURN_DRIVE_TO_AMP = 0.0;

  public static final double P_Y_DRIVE_TO_AMP = 0.01;
  public static final double I_Y_DRIVE_TO_AMP = 0.001;
  public static final double D_Y_DRIVE_TO_AMP = 0.0;

  public static final double P_X_DRIVE_TO_AMP = 0.01;
  public static final double I_X_DRIVE_TO_AMP = 0.001;
  public static final double D_X_DRIVE_TO_AMP = 0.0;

  public enum ScorePosition {
    HIGH,
    MIDDLE,
  }
}

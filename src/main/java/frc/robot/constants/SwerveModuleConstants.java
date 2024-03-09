package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {

  public static final double DRIVE_POSITION_CONVERSION_FACTOR =
      (RobotConstants.WHEEL_DIAMETER * Math.PI) * DriveConstants.DRIVE_GEAR_REDUCTION; // meters
  public static final double DRIVE_VELOCITY_CONVERSION_FACTOR =
      ((RobotConstants.WHEEL_DIAMETER * Math.PI) * DriveConstants.DRIVE_GEAR_REDUCTION)
          / 60.0; // meters per second

  public static final double PIVOT_POSITION_CONVERSION_FACTOR =
      2 * Math.PI * DriveConstants.STEER_DRIVE_REDUCTION;
  public static final double PIVOT_VELOCITY_CONVERSION_FACTOR =
      (2 * Math.PI) / 60 * DriveConstants.STEER_DRIVE_REDUCTION;

  public static class Test {
    public static final double VELOCITY = 2.5;
    public static final double VELOCITY_TOLERANCE = .1;
    public static final double WAIT = 2.5;
    public static final double TIMEOUT = WAIT + 1;
  }

  public static class FrontLeftConstants {
    public static final String MODULE_NAME = "FL";

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 31;
      public static final int CURRENT_LIMT = 40;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = DRIVE_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.01;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_MPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 9;
      public static final int ABSOLUTE_ENCODER_PORT = 23;
      public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromDegrees(-176.04);
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = PIVOT_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = PIVOT_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.9;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class FrontRightConstants {
    public static final String MODULE_NAME = "FR";

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 18;
      public static final int CURRENT_LIMT = 40;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = DRIVE_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.01;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_MPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 10;
      public static final int ABSOLUTE_ENCODER_PORT = 21;
      public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromDegrees(-80.5);
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = PIVOT_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = PIVOT_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.9;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class BackLeftConstants {
    public static final String MODULE_NAME = "BL";

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 30;
      public static final int CURRENT_LIMT = 40;
      public static final boolean INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = DRIVE_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.01;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_MPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 2;
      public static final int ABSOLUTE_ENCODER_PORT = 22;
      public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromDegrees(-52.2);
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = PIVOT_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = PIVOT_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.9;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }

  public static class BackRightConstants {
    public static final String MODULE_NAME = "BR";

    public static class DriveMotor {
      public static final String NAME = MODULE_NAME + " Drive";
      public static final int MOTOR_PORT = 19;
      public static final int CURRENT_LIMT = 40;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = DRIVE_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.01;
        public static final double Ff = 1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_MPS;
      }
    }

    public static class PivotMotor {
      public static final String NAME = MODULE_NAME + " Pivot";
      public static final int MOTOR_PORT = 17;
      public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromDegrees(-159.43);
      public static final int ABSOLUTE_ENCODER_PORT = 24;
      public static final int CURRENT_LIMT = 20;
      public static final boolean INVERTED = true;
      public static final boolean ENCODER_INVERTED = false;
      public static final double POSITION_CONVERSION_FACTOR = PIVOT_POSITION_CONVERSION_FACTOR;
      public static final double VELOCITY_CONVERSION_FACTOR = PIVOT_VELOCITY_CONVERSION_FACTOR;

      public static class MotorPid {
        public static final double P = 0.9;
        public static final boolean POSITION_PID_WRAPPING_ENABLED = true;
        public static final int POSITION_PID_WRAPPING_MIN = 0;
        public static final double POSITION_PID_WRAPPING_MAX = 2 * Math.PI;
      }
    }
  }
}

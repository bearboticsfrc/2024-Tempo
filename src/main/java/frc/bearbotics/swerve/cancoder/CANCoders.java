package frc.bearbotics.swerve.cancoder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CTREUtil;
import java.util.HashMap;
import java.util.Map;

public class CANCoders {
  private final double CONFIGURATION_TIMEOUT =
      Units.millisecondsToSeconds(10000); // TOOD: Move to constants
  private final double UPDATE_TIMEOUT = Units.millisecondsToSeconds(1000);

  private static CANCoders instance;

  private Map<Integer, CANcoder> cancoders = new HashMap<Integer, CANcoder>();

  public static CANCoders getInstance() {
    if (instance == null) {
      instance = new CANCoders();
    }

    return instance;
  }

  public void configure(CANCoderBuilder cancoderConfiguration) {
    CANcoder cancoder = new CANcoder(cancoderConfiguration.getId());
    CANcoderConfiguration canCoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withMagnetOffset(cancoderConfiguration.getOffsetDegrees().getDegrees() / 360)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    for (int attempt = 1; attempt < 6; attempt++) {
      StatusCode statusCode =
          CTREUtil.checkCtreError(
              cancoder.getConfigurator().apply(canCoderConfig, CONFIGURATION_TIMEOUT));

      if (statusCode == StatusCode.TxTimeout) {
        DriverStation.reportError(
            String.format(
                "[CANCoder %s]: Configuration timed out on attempt %s",
                cancoderConfiguration.getId()),
            false);
      }

      if (statusCode.isError()) {
        DriverStation.reportError(
            String.format(
                "[CANCoder %s]: Configuration errored out with description \"%s\" on attempt %s",
                cancoderConfiguration.getId(), statusCode.getDescription(), attempt),
            false);
      } else {
        break;
      }
    }

    while (!cancoder.getAbsolutePosition().waitForUpdate(1).hasUpdated()) {
      DriverStation.reportError(
          String.format(
              "[CANCoder %s]: Timed out while waiting for update... sleeping and retrying",
              cancoderConfiguration.getId()),
          false);
      Timer.delay(0.25); // TOOD: Might need a maximum attempts
    }

    cancoders.put(cancoderConfiguration.getId(), cancoder);
  }

  public CANcoder get(int canCoderId) {
    return cancoders.get(canCoderId);
  }

  private CANCoders() {}

  public static class CANCoderBuilder {
    private int id;
    private Rotation2d offsetDegrees;

    public int getId() {
      return id;
    }

    public CANCoderBuilder setId(int id) {
      this.id = id;
      return this;
    }

    public Rotation2d getOffsetDegrees() {
      return offsetDegrees;
    }

    public CANCoderBuilder setOffsetDegrees(Rotation2d offsetDegrees) {
      this.offsetDegrees = offsetDegrees;
      return this;
    }
  }
}

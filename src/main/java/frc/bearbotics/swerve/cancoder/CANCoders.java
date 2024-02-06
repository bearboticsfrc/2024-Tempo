package frc.bearbotics.swerve.cancoder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CTREUtil;
import java.util.HashMap;
import java.util.Map;

public class CANCoders {
  private final double CONFIGURATION_TIMEOUT = 10; // TOOD: Move to constants

  private static CANCoders instance;
  private Map<Integer, ObservedCANCoder> cancoders = new HashMap<Integer, ObservedCANCoder>();

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
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                    .withMagnetOffset(cancoderConfiguration.getOffsetDegrees().getDegrees())
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    double startTime = Timer.getFPGATimestamp();

    for (int attempt = 1; attempt < 6; attempt++) {
      StatusCode statusCode =
          CTREUtil.checkCtreError(cancoder.getConfigurator().apply(canCoderConfig, 100));

      if ((Timer.getFPGATimestamp()) - startTime >= CONFIGURATION_TIMEOUT) {
        DriverStation.reportError(
            "[CANCoder]: Configuration timed out on attempt " + attempt, false);
        break;
      }

      if (statusCode.isError()) {
        DriverStation.reportError(
            "[CANCoder]: Configuration errored out with status code "
                + statusCode.value
                + " on attempt "
                + attempt,
            false);
      } else {
        break;
      }
    }

    cancoders.put(
        cancoderConfiguration.getId(), new ObservedCANCoder(cancoder, new CANObserver(cancoder)));
  }

  public boolean allHaveBeenInitialized() {
    for (ObservedCANCoder cancoder : cancoders.values()) {
      if (!cancoder.getObserver().hasUpdate()) {
        return false;
      }
    }

    return true;
  }

  public ObservedCANCoder get(int canCoderId) {
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

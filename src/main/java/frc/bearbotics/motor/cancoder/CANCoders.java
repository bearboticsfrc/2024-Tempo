package frc.bearbotics.motor.cancoder;

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

/** Singleton class for managing and configuring CANCoders in a robotics application. */
public class CANCoders {
  // Timeout for configuring CANCoders (milliseconds)
  private final double CONFIGURATION_TIMEOUT = Units.millisecondsToSeconds(10000);

  // Timeout for updating CANCoders (milliseconds)
  private final double UPDATE_TIMEOUT = Units.millisecondsToSeconds(1000);

  // Singleton instance
  private static CANCoders instance;

  // Map to store CANCoders based on their IDs
  private Map<Integer, CANcoder> cancoders = new HashMap<Integer, CANcoder>();

  /** Private constructor to enforce singleton pattern. */
  private CANCoders() {}

  /**
   * Gets the singleton instance of the CANCoders class.
   *
   * @return The singleton instance of CANCoders.
   */
  public static CANCoders getInstance() {
    if (instance == null) {
      instance = new CANCoders();
    }
    return instance;
  }

  /**
   * Configures a CANCoder based on the provided CANCoderBuilder configuration.
   *
   * @param cancoderConfiguration The configuration for the CANCoder.
   */
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
                cancoderConfiguration.getId(), attempt),
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

    while (!cancoder.getAbsolutePosition().waitForUpdate(UPDATE_TIMEOUT).hasUpdated()) {
      DriverStation.reportError(
          String.format(
              "[CANCoder %s]: Timed out while waiting for update... sleeping and retrying",
              cancoderConfiguration.getId()),
          false);
      Timer.delay(0.25);
    }

    cancoders.put(cancoderConfiguration.getId(), cancoder);
  }

  /**
   * Gets a configured CANCoder based on its ID.
   *
   * @param canCoderId The ID of the CANCoder.
   * @return The configured CANCoder.
   */
  public CANcoder get(int canCoderId) {
    return cancoders.get(canCoderId);
  }

  /** Builder class for constructing instances of the {@link CANCoders} class. */
  public static class CANCoderBuilder {
    // ID of the CANCoder
    private int id;

    // Offset in degrees for the CANCoder
    private Rotation2d offsetDegrees;

    /**
     * Gets the ID of the CANCoder.
     *
     * @return The ID of the CANCoder.
     */
    public int getId() {
      return id;
    }

    /**
     * Sets the ID of the CANCoder.
     *
     * @param id The ID to set.
     * @return The current instance of the builder for method chaining.
     */
    public CANCoderBuilder setId(int id) {
      this.id = id;
      return this;
    }

    /**
     * Gets the offset in degrees for the CANCoder.
     *
     * @return The offset in degrees for the CANCoder.
     */
    public Rotation2d getOffsetDegrees() {
      return offsetDegrees;
    }

    /**
     * Sets the offset in degrees for the CANCoder.
     *
     * @param offsetDegrees The offset in degrees to set.
     * @return The current instance of the builder for method chaining.
     */
    public CANCoderBuilder setOffsetDegrees(Rotation2d offsetDegrees) {
      this.offsetDegrees = offsetDegrees;
      return this;
    }
  }
}

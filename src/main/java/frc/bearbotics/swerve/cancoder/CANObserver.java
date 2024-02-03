package frc.bearbotics.swerve.cancoder;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.Optional;

public class CANObserver {
  private final int MINIMUM_VALID_UPDATES = 10; // TODO: Move to constants

  private final CANcoder cancoder;
  private Optional<Timestamp> lastTimestamp = Optional.empty();
  private int validUpdates = 0;

  public CANObserver(CANcoder cancoder) {
    this.cancoder = cancoder;
  }

  public boolean hasUpdate() {
    Timestamp timestamp = cancoder.getAbsolutePosition().getTimestamp();

    if (timestamp.getTime() > lastTimestamp.get().getTime()) {
      validUpdates++;
    }

    lastTimestamp = Optional.of(timestamp);

    return validUpdates > MINIMUM_VALID_UPDATES;
  }
}

package frc.bearbotics.swerve.can;

import com.ctre.phoenix6.hardware.CANcoder;

public class ObservedCANCoder {
  private CANcoder cancoder;
  private CANObserver observer;

  public ObservedCANCoder(CANcoder cancoder, CANObserver observer) {
    this.cancoder = cancoder;
    this.observer = observer;
  }

  public CANcoder getCancoder() {
    return cancoder;
  }

  public CANObserver getObserver() {
    return observer;
  }
}

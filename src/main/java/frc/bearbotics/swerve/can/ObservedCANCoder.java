package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.subsystems.CANObserver;

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

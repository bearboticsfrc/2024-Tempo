package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CandleConstants;
import java.util.function.BooleanSupplier;

public class CandleSubsystem extends SubsystemBase {
  /** The CANdle device instance used to control the LEDs. */
  private final CANdle CANDLE = new CANdle(CandleConstants.PORT);

  /** Represents the entire strip of LEDs as a single segment. */
  private final CandleSegment entireSegment;

  /**
   * Initializes a new instance of the CandleSubsystem, configuring the CANdle device with default
   * settings and preparing the LED segment for control.
   */
  public CandleSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;

    CANDLE.configAllSettings(candleConfiguration, 100);

    this.entireSegment = new CandleSegment(CANDLE, 0, 51, 0);
  }

  /**
   * Clears the animations and sets the color of all LED segments to black (effectively turning them
   * off).
   */
  public void clearSegments() {
    clearSegment(entireSegment);
  }

  /**
   * Clears the animations and sets the color of a specific LED segment to black.
   *
   * @param segment The LED segment to clear.
   */
  public void clearSegment(CandleSegment segment) {
    segment.clearAnimation();
    segment.setColor(Color.kBlack);
  }

  /**
   * Sets the color of the entire LED strip.
   *
   * @param color The color to set the LEDs to.
   */
  public void setColor(Color color) {
    clearSegment(entireSegment);
    entireSegment.setColor(color);
  }

  /**
   * Sets a specific animation pattern with a specified color for the entire LED strip. Supports
   * strobe and larson patterns.
   *
   * @param pattern The pattern to display on the LEDs.
   * @param color The color to use for the pattern.
   */
  public void setPattern(CandlePattern pattern, Color color) {
    clearSegment(entireSegment);

    switch (pattern) {
      case STROBE:
        entireSegment.setStrobeAnimation(color, 0.5);
        break;
      case LARSON:
        entireSegment.setLarsonAnimation(color, 0.1);
        break;
    }
  }

  /**
   * Sets the LED color based on the robot's alliance color. Intended to visually indicate the
   * alliance during competitions.
   *
   * @param isRedAlliance A BooleanSupplier that returns true if the robot is on the red alliance.
   */
  public void setAllianceColor(BooleanSupplier isRedAlliance) {
    if (isRedAlliance.getAsBoolean()) {
      setColor(Color.kRed);
    } else {
      setColor(Color.kBlue);
    }
  }
}

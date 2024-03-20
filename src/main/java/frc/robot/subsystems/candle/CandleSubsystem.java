package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.fms.AllianceReadyListener;
import frc.robot.constants.CandleConstants;

public class CandleSubsystem extends SubsystemBase implements AllianceReadyListener {
  /** The CANdle device instance used to control the LEDs. */
  private final CANdle CANDLE = new CANdle(CandleConstants.PORT);

  /** Represents the entire strip of LEDs as a single segment. */
  private final CandleSegment entireSegment = new CandleSegment(CANDLE, 8, 99, 0);

  /** Represents the back strip of LEDs as a two segments. */
  private final CandleSegment backTopSegment = new CandleSegment(CANDLE, 0, 4, 1);

  private final CandleSegment backBottomSegment = new CandleSegment(CANDLE, 4, 4, 2);

  /**
   * Initializes a new instance of the CandleSubsystem, configuring the CANdle device with default
   * settings and preparing the LED segment for control.
   */
  public CandleSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    CANDLE.configAllSettings(candleConfiguration, 100);

    AllianceColor.addListener(this);
  }

  @Override
  public void updateAllianceColor(Alliance alliance) {
    Color currentColor = entireSegment.getColor();

    if (currentColor == null || currentColor == Color.kRed || currentColor == Color.kBlue) {
      setAllianceColor();
    }
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
    setColor(color, entireSegment);
  }

  /**
   * Sets the color of the specified segment.
   *
   * @param color The color to set the LEDs to.
   * @param segment The segment to set the color to.
   */
  public void setColor(Color color, CandleSegment segment) {
    clearSegment(segment);
    segment.setColor(color);
  }

  /**
   * Sets a specific animation pattern with a specified color for the entire LED strip. Supports
   * strobe and larson patterns.
   *
   * @param pattern The pattern to display on the LEDs.
   * @param color The color to use for the pattern.
   */
  public void setPattern(CandlePattern pattern, Color color) {
    setPattern(pattern, color, entireSegment);
  }

  /**
   * Sets a specific animation pattern with a specified color for specified segment. Supports strobe
   * and larson patterns.
   *
   * @param pattern The pattern to display on the LEDs.
   * @param color The color to use for the pattern.
   * @param segment The segment to set the pattern to.
   */
  public void setPattern(CandlePattern pattern, Color color, CandleSegment segment) {
    clearSegment(segment);

    switch (pattern) {
      case STROBE:
        segment.setStrobeAnimation(color, 10);
        break;
      case LARSON:
        segment.setLarsonAnimation(color, 0.001);
        break;
    }
  }

  /**
   * Sets the LED color based on the robot's alliance color. Intended to visually indicate the
   * alliance during competitions.
   */
  public void setAllianceColor() {
    Color color = getAllianceColor(AllianceColor.getAlliance().get());

    setColor(color);
    setPattern(CandlePattern.LARSON, color, backTopSegment);
    setPattern(CandlePattern.LARSON, color, backBottomSegment);
  }

  private Color getAllianceColor(Alliance alliance) {
    return alliance == Alliance.Red ? Color.kRed : Color.kBlue;
  }
}

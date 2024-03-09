/**
 * The BlinkinSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.CandleConstants;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The CandleSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
public class CandleSubsystem extends SubsystemBase {

  private static final CANdle candle = new CANdle(CandleConstants.CANDLE_PORT);

  // Constants for predefined colors
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color RED = new Color(255, 0, 0);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color PURPLE = new Color(125, 18, 255);
  public static final Color YELLOW = new Color(280, 130, 0);
  public static final Color BLACK = new Color(0, 0, 0);

  /** Constructor for the CandleSubsystem. Configures CANdle settings upon initialization. */
  public CandleSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
  }

  /**
   * Sets the brightness of the LED strip.
   *
   * @param percent The brightness percentage (0 to 100).
   */
  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  /** Resets the LED segments to their default state. */
  public void reset() {
    clearSegment(LEDSegment.CANdle);
    clearSegment(LEDSegment.MainStrip);
    signalAllianceColor();
  }

  /** Sets the color of the MainStrip segment to purple. */
  public void setColorPurple() {
    LEDSegment.MainStrip.setColor(PURPLE);
  }

  /** Sets the color of the MainStrip segment to yellow. */
  public void setColorYellow() {
    LEDSegment.MainStrip.setColor(YELLOW);
  }

  /** Turns off the MainStrip segment. */
  public void turnOff() {
    LEDSegment.MainStrip.disableLEDs();
  }

  /**
   * Clears animations and disables LEDs for a specific LED segment.
   *
   * @param segment The LED segment to clear.
   */
  public void clearSegment(LEDSegment segment) {
    segment.clearAnimation();
    segment.disableLEDs();
  }

  /** Signals a specific source by activating a strobe animation on the MainStrip segment. */
  public void signalSource() {
    LEDSegment.MainStrip.setStrobeAnimation(YELLOW, 10);
  }

  /**
   * Signals that a note is in the holder by setting the color of the MainStrip segment to green.
   */
  public void signalNoteInHolder() {
    LEDSegment.MainStrip.setColor(GREEN);
  }

  /** Signals a note on the right side by setting the color of the RightSide segment to purple. */
  public void signalNoteRight() {
    LEDSegment.RightSide.setColor(PURPLE);
  }

  /** Signals a note on the left side by setting the color of the LeftSide segment to purple. */
  public void signalNoteLeft() {
    LEDSegment.LeftSide.setColor(PURPLE);
  }

  /** Signals a note in the middle by setting the color of the MiddleSide segment to purple. */
  public void signalNoteStraight() {
    LEDSegment.MiddleSide.setColor(PURPLE);
  }

  /**
   * Signals a note based on the position of a tracked target.
   *
   * @param note The tracked target representing the note.
   */
  public void signalNote(PhotonTrackedTarget note) {
    Rotation2d angle = Rotation2d.fromDegrees(-note.getYaw());

    if (angle.getDegrees() < -8) {
      signalNoteLeft();
    } else if (angle.getDegrees() > 8) {
      signalNoteRight();
    } else {
      signalNoteStraight();
    }
  }

  /** Signals the alliance color by setting the colors of the MainStrip and CANdle segments. */
  public void signalAllianceBlue() {
    LEDSegment.MainStrip.setColor(BLUE);
    LEDSegment.CANdle.setColor(BLUE);
  }

  /** Signals the alliance color by setting the colors of the MainStrip and CANdle segments. */
  public void signalAllianceRed() {
    LEDSegment.MainStrip.setColor(RED);
    LEDSegment.CANdle.setColor(RED);
  }

  /**
   * Signals the alliance color based on the current alliance. If the alliance is red, signals red;
   * otherwise, signals blue.
   */
  public void signalAllianceColor() {
    if (AllianceColor.isRedAlliance()) {
      signalAllianceRed();
      return;
    }

    signalAllianceBlue();
  }

  public static enum LEDSegment {
    CANdle(0, 8, 0),
    MainStrip(8, 99, 1),
    LeftSide(8, 33, 2),
    MiddleSide(8 + 33, 33, 3),
    RightSide(66 + 8, 33, 4);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    // private enum constructor
    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(BLACK);
    }

    public void setLarsonAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 7));
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, 0.5, segmentSize, false, startIndex));
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}

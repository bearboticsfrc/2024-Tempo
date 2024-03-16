/**
 * Represents a segment of LEDs on a CANdle device, providing methods to control colors and
 * animations for that segment. This class encapsulates the functionality to set static colors,
 * clear animations, and apply predefined animations to a specified range of LEDs.
 */
package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;

public class CandleSegment {
  private final CANdle candle;
  private final int startIndex;
  private final int segmentSize;
  private final int animationSlot;

  /**
   * Constructs a new CandleSegment object to control a segment of LEDs on a CANdle device.
   *
   * @param candle The CANdle device this segment is part of.
   * @param startIndex The starting index of this LED segment on the CANdle LED strip.
   * @param segmentSize The number of LEDs in this segment.
   * @param animationSlot The animation slot to use for this segment's animations.
   */
  public CandleSegment(CANdle candle, int startIndex, int segmentSize, int animationSlot) {
    this.candle = candle;
    this.startIndex = startIndex;
    this.segmentSize = segmentSize;
    this.animationSlot = animationSlot;
  }

  /** Clears any ongoing animation on this LED segment. */
  public void clearAnimation() {
    candle.clearAnimation(animationSlot);
  }

  /**
   * Sets a static color for this LED segment. All LEDs in the segment will display the specified
   * color.
   *
   * @param color The color to set the LEDs to. Uses the WPILib Color class.
   */
  public void setColor(Color color) {
    candle.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        0,
        startIndex,
        segmentSize);
  }

  /**
   * Applies a Larson (Knight Rider/KITT car effect) animation to this LED segment.
   *
   * @param color The color of the Larson scanner effect.
   * @param speed The speed of the Larson scanner effect.
   */
  public void setLarsonAnimation(Color color, double speed) {
    LarsonAnimation animation =
        new LarsonAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            segmentSize,
            LarsonAnimation.BounceMode.Back,
            1);

    setAnimation(animation, animationSlot);
  }

  /**
   * Applies a strobe animation to this LED segment.
   *
   * @param color The color of the strobe effect.
   * @param speed The speed of the strobe effect.
   */
  public void setStrobeAnimation(Color color, double speed) {
    StrobeAnimation animation =
        new StrobeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            segmentSize,
            startIndex);

    setAnimation(animation, animationSlot);
  }

  /**
   * Helper method to apply an animation to this LED segment.
   *
   * @param animation The animation object to apply.
   * @param slot The animation slot to use, typically matches the animationSlot provided in
   *     constructor.
   */
  private void setAnimation(Animation animation, int slot) {
    candle.animate(animation, animationSlot);
  }
}

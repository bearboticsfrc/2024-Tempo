/**
 * Defines the set of predefined patterns that can be applied to the Candle subsystem's LED strip.
 * This enum facilitates easy selection of lighting effects to enhance the robot's visual feedback
 * or aesthetic appeal.
 */
package frc.robot.subsystems.candle;

public enum CandlePattern {
  /** A strobing effect, rapidly blinking the LEDs on and off. */
  STROBE,

  /**
   * A "Larson scanner" (or Knight Rider) effect, creating a moving dot back and forth across the
   * LEDs.
   */
  LARSON;
}

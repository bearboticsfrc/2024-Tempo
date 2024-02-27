/**
 * The BlinkinSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.LightsConstants;
import java.util.List;
import java.util.function.BooleanSupplier;

public class BlinkinSubsystem extends SubsystemBase {

  private final BooleanSupplier isBlue =
      () -> (AllianceColor.alliance == Alliance.Blue) ? true : false;
  private final List<Spark> blinkins =
      List.of(new Spark(LightsConstants.FRONT_BLINKIN), new Spark(LightsConstants.BACK_BLINKIN));

  /**
   * Constructs the BlinkinSubsystem and initializes the Blinkin LED controllers. Resets the Blinkin
   * LEDs to the alliance color.
   */
  public BlinkinSubsystem() {
    reset();
  }

  /**
   * Sets the color of the specified list of Blinkin LED controllers.
   *
   * @param blinkins The list of Spark controllers representing Blinkin LEDs.
   * @param color The desired color from the LightsConstants.Color enum.
   */
  public void setColor(List<Spark> blinkins, LightsConstants.Color color) {
    for (Spark blinkin : blinkins) {
      blinkin.set(color.value);
    }
  }

  /**
   * Sets the color of a single Blinkin LED controller.
   *
   * @param blinkin The Spark controller representing a Blinkin LED.
   * @param color The desired color from the LightsConstants.Color enum.
   */
  public void setColor(Spark blinkin, LightsConstants.Color color) {
    blinkin.set(color.value);
  }

  /**
   * Sets the pattern of the specified list of Blinkin LED controllers.
   *
   * @param blinkins The list of Spark controllers representing Blinkin LEDs.
   * @param blinkinPattern The desired pattern from the LightsConstants.BlinkinPattern enum.
   */
  public void setPattern(List<Spark> blinkins, LightsConstants.BlinkinPattern blinkinPattern) {
    for (Spark blinkin : blinkins) {
      blinkin.set(blinkinPattern.value);
    }
  }

  /**
   * Sets the pattern of a single Blinkin LED controller.
   *
   * @param blinkin The Spark controller representing a Blinkin LED.
   * @param blinkinPattern The desired pattern from the LightsConstants.BlinkinPattern enum.
   */
  public void setPattern(Spark blinkin, LightsConstants.BlinkinPattern blinkinPattern) {
    blinkin.set(blinkinPattern.value);
  }

  /** Signals the Blinkin LED controllers to display a strobing gold pattern. */
  public void signalSource() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.STROBE_GOLD);
  }

  /** Sets the color of the Blinkin LED controllers to blue. */
  public void setBlue() {
    setColor(blinkins, LightsConstants.Color.BLUE);
  }

  /** Sets the color of the Blinkin LED controllers to red. */
  public void setRed() {
    setColor(blinkins, LightsConstants.Color.RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a red heartbeat animation. */
  public void setRedAutoAnimation() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.HEARTBEAT_RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a blue heartbeat animation. */
  public void setBlueAutoAnimation() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.HEARTBEAT_BLUE);
  }

  /**
   * Signals the Blinkin LED controllers to display a green color (used for a note in holder
   * signal).
   */
  public void signalNoteInHolder() {
    setColor(blinkins, LightsConstants.Color.GREEN);
  }

  /**
   * Displays the alliance color in autonomous mode by selecting the appropriate animation based on
   * the alliance color.
   */
  public void displayAllianceColorInAuto() {
    if (isBlue.getAsBoolean() == true) {
      setBlueAutoAnimation();
      return;
    }
    setRedAutoAnimation();
  }

  /**
   * Displays the alliance color in teleop mode by setting the color based on the alliance color.
   */
  public void displayAllianceColor() {
    if (isBlue.getAsBoolean() == true) {
      setBlue();
      return;
    }
    setRed();
  }

  /** Resets the Blinkin LED controllers based on the operating mode (teleop or autonomous). */
  public void reset() {
    if (DriverStation.isTeleop()) {
      displayAllianceColor();
      return;
    }
    displayAllianceColorInAuto();
  }
}

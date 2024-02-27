/**
 * The BlinkinSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.BlinkinConstants;
import java.util.List;

public class BlinkinSubsystem extends SubsystemBase {
  private final List<Spark> blinkins =
      List.of(new Spark(BlinkinConstants.FRONT_BLINKIN), new Spark(BlinkinConstants.BACK_BLINKIN));

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
   * @param color The desired color from the BlinkinConstants.Color enum.
   */
  public void setColor(List<Spark> blinkins, BlinkinConstants.Color color) {
    for (Spark blinkin : blinkins) {
      blinkin.set(color.value);
    }
  }

  /**
   * Sets the color of a single Blinkin LED controller.
   *
   * @param blinkin The Spark controller representing a Blinkin LED.
   * @param color The desired color from the BlinkinConstants.Color enum.
   */
  public void setColor(Spark blinkin, BlinkinConstants.Color color) {
    blinkin.set(color.value);
  }

  /**
   * Sets the pattern of the specified list of Blinkin LED controllers.
   *
   * @param blinkins The list of Spark controllers representing Blinkin LEDs.
   * @param blinkinPattern The desired pattern from the BlinkinConstants.BlinkinPattern enum.
   */
  public void setPattern(List<Spark> blinkins, BlinkinConstants.Pattern blinkinPattern) {
    for (Spark blinkin : blinkins) {
      blinkin.set(blinkinPattern.value);
    }
  }

  /**
   * Sets the pattern of a single Blinkin LED controller.
   *
   * @param blinkin The Spark controller representing a Blinkin LED.
   * @param blinkinPattern The desired pattern from the BlinkinConstants.BlinkinPattern enum.
   */
  public void setPattern(Spark blinkin, BlinkinConstants.Pattern blinkinPattern) {
    blinkin.set(blinkinPattern.value);
  }

  /** Signals the Blinkin LED controllers to display a strobing gold pattern. */
  public void signalSource() {
    setPattern(blinkins, BlinkinConstants.Pattern.STROBE_GOLD);
  }

  /** Sets the color of the Blinkin LED controllers to blue. */
  public void setBlue() {
    setColor(blinkins, BlinkinConstants.Color.BLUE);
  }

  /** Sets the color of the Blinkin LED controllers to red. */
  public void setRed() {
    setColor(blinkins, BlinkinConstants.Color.RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a red heartbeat animation. */
  public void setRedAutoAnimation() {
    setPattern(blinkins, BlinkinConstants.Pattern.HEARTBEAT_RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a blue heartbeat animation. */
  public void setBlueAutoAnimation() {
    setPattern(blinkins, BlinkinConstants.Pattern.HEARTBEAT_BLUE);
  }

  /**
   * Signals the Blinkin LED controllers to display a green color (used for a note in holder
   * signal).
   */
  public void signalNoteInHolder() {
    setColor(blinkins, BlinkinConstants.Color.GREEN);
  }

  /**
   * Displays the alliance color in autonomous mode by selecting the appropriate animation based on
   * the alliance color.
   */
  public void displayAllianceColorInAuto() {
    if (AllianceColor.isRedAlliance()) {
      setRedAutoAnimation();
      return;
    }
    setBlueAutoAnimation();
  }

  /**
   * Displays the alliance color in teleop mode by setting the color based on the alliance color.
   */
  public void displayAllianceColor() {
    if (AllianceColor.isRedAlliance()) {
      setRed();
      return;
    }
    setBlue();
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

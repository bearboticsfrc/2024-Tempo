/**
 * The BlinkinSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.BlinkinConstants;
import java.util.List;
import java.util.function.BooleanSupplier;

public class BlinkinSubsystem extends SubsystemBase {
  private final BooleanSupplier blueSupplier =
      () -> AllianceColor.alliance == Alliance.Blue ? true : false;

  private final BooleanSupplier isNoteInFeeder;
  private final List<PWM> blinkinsPWM =
      List.of(new PWM(BlinkinConstants.FRONT_BLINKIN), new PWM(BlinkinConstants.BACK_BLINKIN));

  /**
   * Constructs the BlinkinSubsystem and initializes the Blinkin LED controllers. Resets the Blinkin
   * LEDs to the alliance color.
   */
  public BlinkinSubsystem(BooleanSupplier isNoteInFeeder) {
    this.isNoteInFeeder = isNoteInFeeder;
  }

  /**
   * Sets the color of the specified list of Blinkin LED controllers.
   *
   * @param blinkins The list of Spark controllers representing Blinkin LEDs.
   * @param color The desired color from the BlinkinConstants.Color enum.
   */
  public void setColor(BlinkinConstants.Color color) {
    for (PWM pwm : this.blinkinsPWM) {
      pwm.setPulseTimeMicroseconds(2125);
      pwm.setSpeed(BlinkinConstants.Pattern.CP1_STROBE.value);

      pwm.setSpeed(color.value);
    }
  }

  /**
   * Sets the pattern of the specified list of Blinkin LED controllers.
   *
   * @param blinkins The list of Spark controllers representing Blinkin LEDs.
   * @param blinkinPattern The desired pattern from the BlinkinConstants.BlinkinPattern enum.
   */
  public void setPattern(BlinkinConstants.Pattern blinkinPattern) {
    for (PWM pwm : this.blinkinsPWM) {
      pwm.setPulseTimeMicroseconds(2125);
      pwm.setSpeed(BlinkinConstants.Pattern.CP1_STROBE.value);
      pwm.setSpeed(blinkinPattern.value);
    }
  }

  /** Signals the Blinkin LED controllers to display a strobing gold pattern. */
  public void signalSource() {
    setPattern(BlinkinConstants.Pattern.STROBE_GOLD);
  }

  /** Sets the color of the Blinkin LED controllers to blue. */
  public void setBlue() {
    setColor(BlinkinConstants.Color.BLUE);
  }

  /** Sets the color of the Blinkin LED controllers to red. */
  public void setRed() {
    setColor(BlinkinConstants.Color.RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a red heartbeat animation. */
  public void setRedAutoAnimation() {
    setPattern(BlinkinConstants.Pattern.HEARTBEAT_RED);
  }

  /** Sets the pattern of the Blinkin LED controllers to a blue heartbeat animation. */
  public void setBlueAutoAnimation() {
    setPattern(BlinkinConstants.Pattern.HEARTBEAT_BLUE);
  }

  /**
   * Signals the Blinkin LED controllers to display a green color (used for a note in holder
   * signal).
   */
  public void signalNoteInHolder() {
    setColor(BlinkinConstants.Color.GREEN);
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
    if (blueSupplier.getAsBoolean()) {
      setBlue();
      return;
    }
    setRed();
  }

  /** Resets the Blinkin LED controllers based on the operating mode (teleop or autonomous). */
  public void reset() {

    if (isNoteInFeeder.getAsBoolean() == true) {
      signalNoteInHolder();
      return;
    } else if (DriverStation.isTeleop()) {
      displayAllianceColor();
      return;
    }
    displayAllianceColorInAuto();
  }
}

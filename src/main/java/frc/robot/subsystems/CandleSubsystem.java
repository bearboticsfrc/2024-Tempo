/**
 * The BlinkinSubsystem class represents the subsystem responsible for controlling the Blinkin LED
 * controllers on the robot. It extends the WPILib's SubsystemBase class and provides methods for
 * setting colors and patterns of the Blinkin LEDs.
 */
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.CandleConstants;
import java.util.function.BooleanSupplier;

public class CandleSubsystem extends SubsystemBase {

  private static final CANdle candle = new CANdle(CandleConstants.CANDLE_PORT);
  public static final BooleanSupplier allianceBlue =
      () -> (AllianceColor.alliance == Alliance.Blue) ? true : false;

  public static final Color blue = new Color(0, 0, 255);
  public static final Color red = new Color(255, 0, 0);
  public static final Color green = new Color(0, 255, 0);
  public static final Color purple = new Color(125, 18, 255);
  public static final Color yellow = new Color(280, 130, 0);
  public static final Color black = new Color(0, 0, 0);

  public CandleSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = false;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public void reset() {
    clearSegment(LEDSegment.CANdle);
    clearSegment(LEDSegment.MainStrip);
    signalAllianceColor();
  }

  public void setColorPurple() {
    LEDSegment.MainStrip.setColor(purple);
  }

  public void setColorYellow() {
    LEDSegment.MainStrip.setColor(yellow);
  }

  public void turnOff() {
    LEDSegment.MainStrip.disableLEDs();
  }

  public void clearSegment(LEDSegment segment) {
    segment.clearAnimation();
    segment.disableLEDs();
  }

  public void signalSource() {
    LEDSegment.MainStrip.setStrobeAnimation(yellow, 10);
  }

  public void signalNoteInHolder() {
    LEDSegment.MainStrip.setColor(green);
  }

  public void signalNoteRight() {
    reset();
    LEDSegment.RightSide.setColor(purple);
  }

  public void signalNoteLeft() {
    reset();
    LEDSegment.LeftSide.setColor(purple);
  }

  public void signalNoteStraight() {
    reset();
    LEDSegment.MiddleSide.setColor(purple);
  }

  public void signalNote(Rotation2d note) {
    reset();
    if (note.getDegrees() < -8) {
      signalNoteLeft();
    } else if (note.getDegrees() > 8) {
      signalNoteRight();
    } else {
      signalNoteStraight();
    }
  }

  public void signalAllianceBlue() {

    LEDSegment.MainStrip.setColor(blue);
    LEDSegment.CANdle.setColor(blue);
  }

  public void signalAllianceRed() {
    LEDSegment.MainStrip.setColor(red);
    LEDSegment.CANdle.setColor(red);
  }

  public void signalAllianceColor() {
    if (allianceBlue.getAsBoolean()) {
      signalAllianceBlue();
      return;
    }
    signalAllianceRed();
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
      setColor(black);
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

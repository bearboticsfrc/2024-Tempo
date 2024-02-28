package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;
import java.util.Dictionary;
import java.util.Hashtable;

public abstract class LightsConstants {
  public static Dictionary<String, Character> strip = new Hashtable<>();
  public static Dictionary<String, Character> color = new Hashtable<>();
  public static final I2C communincation = new I2C(I2C.Port.kOnboard, 0x1);

  public LightsConstants() {
    configureColor();
    configureStrip();
  }

  public static void configureStrip() {
    strip.put("one", '0');
    strip.put("two", '1');
    strip.put("three", '2');
    strip.put("four", '3');
    strip.put("five", '4');
    strip.put("six", '5');
  }

  public static void configureColor() {
    color.put("blank", '0');
    color.put("green", '1');
    color.put("red", '2');
    color.put("blue", '3');
    color.put("purple", '4');
    color.put("teal", '5');
    color.put("white", '6');
  }
}

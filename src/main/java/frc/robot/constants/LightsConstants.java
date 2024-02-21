package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;
import java.util.Dictionary;
import java.util.Hashtable;

public class LightsConstants {
  public static final int ARDUINO_ADDRESS = 1;
  public static Dictionary<String, Integer> strip = new Hashtable<>();
  public static Dictionary<String, Integer> color = new Hashtable<>();
  public static final I2C communincation = new I2C(I2C.Port.kOnboard, 0x4068);

  public static void config() {
    strip.put("one", 00);
    strip.put("two", 10);
    strip.put("three", 20);
    strip.put("four", 30);
    strip.put("five", 40);
    strip.put("six", 50);
    strip.put("left", 60);
    strip.put("right", 70);
    strip.put("forward", 80);
    strip.put("back", 90);
    strip.put("all", 100);
    color.put("blank", 0);
    color.put("green", 1);
    color.put("red", 2);
    color.put("blue", 3);
    color.put("purple", 4);
    color.put("teal", 5);
  }
}

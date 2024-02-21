package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.LightsConstants;

public class LightsSubsystem extends LightsConstants implements Subsystem {
  private static String strip;
  private static String color;

  public LightsSubsystem() {
    this.strip = "one";
    this.color = "blank";
    send();
  }

  public void send() {
    LightsConstants.communincation.write(
        LightsConstants.ARDUINO_ADDRESS,
        (LightsConstants.strip.get(strip)) + (LightsConstants.color.get(color)));
  }

  public void setColor(String color) {
    this.color = color;
    send();
  }

  public void setStrip(String strip) {
    this.strip = strip;
    send();
  }
}

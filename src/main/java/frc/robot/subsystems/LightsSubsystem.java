package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.LightsConstants;

public class LightsSubsystem implements Subsystem {

  public LightsSubsystem() {
    LightsConstants.config();
  }

  public void send(String strip, String color) {
    LightsConstants.communincation.write(
        LightsConstants.ARDUINO_ADDRESS,
        (LightsConstants.strip.get(strip)) + (LightsConstants.color.get(color)));
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.LightsConstants;
import java.util.Stack;

public class LightsSubsystem {
  private final Spark blinkin;
  private Stack<LightsConstants.Color> colorStack = new Stack<LightsConstants.Color>();

  public LightsSubsystem() {
    blinkin = new Spark(LightsConstants.pwmPort);
    set(LightsConstants.Color.BLACK);
  }

  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      blinkin.set(val);
    }
  }

  public void set(LightsConstants.Color color) {
    blinkin.set(color.value);
    colorStack.add(color);
  }

  public LightsConstants.Color getPreviousColor() {
    colorStack.pop();
    return colorStack.pop();
  }
}

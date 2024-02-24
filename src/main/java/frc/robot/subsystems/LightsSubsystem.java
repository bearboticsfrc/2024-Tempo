package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.LightsConstants;

public class LightsSubsystem {
  private final Spark blinkinFront;
  private final Spark blinkinBack;
  private final Spark[] blinkins;

  public LightsSubsystem() {
    blinkinFront = new Spark(LightsConstants.lightPortFront);
    blinkinBack = new Spark(LightsConstants.lightPortBack);
    blinkins = new Spark[] {blinkinFront, blinkinBack};
    setColor(blinkins, LightsConstants.Color.BLACK);
  }

  public void setColor(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      blinkinFront.set(val);
    }
  }

  public void setColor(Spark[] blinkins, LightsConstants.Color color) {
    for (Spark blinkin : blinkins) {
      blinkin.set(color.value);
    }
  }

  public void setColor(Spark blinkin, LightsConstants.Color color) {
    blinkin.set(color.value);
  }

  public void setPattern(Spark[] blinkins, double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      for (Spark blinkin : blinkins) {
        blinkin.set(val);
      }
    }
  }

  public void setPattern(Spark[] blinkins, LightsConstants.BlinkinPattern blinkinPattern) {
    for (Spark blinkin : blinkins) {
      blinkin.set(blinkinPattern.value);
    }
  }

  public void setPattern(Spark blinkin, LightsConstants.BlinkinPattern blinkinPattern) {
    blinkin.set(blinkinPattern.value);
  }
  // public ParallelCommandGroup signalSource(Spark blinkin) {
  // return new ParallelCommandGroup(
  //   new InstantCommand(() -> setColor(blinkins, LightsConstants.Color.YELLOW)),
  // new InstantCommand(() -> setPattern(blinkins, LightsConstants.BlinkinPattern.CP2_STROBE)));
  // }

  public InstantCommand signalSource() {
    return new InstantCommand(() -> setColor(blinkins, LightsConstants.Color.YELLOW));
  }
}

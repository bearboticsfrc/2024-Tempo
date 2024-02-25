package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.LightsConstants;
import java.util.function.BooleanSupplier;

public class LightsSubsystem extends SubsystemBase {

  private final BooleanSupplier isBlue =
      () -> (AllianceColor.alliance == Alliance.Blue) ? true : false;

  private BooleanSupplier isTeleOp;

  private final Spark blinkinFront;
  private final Spark blinkinBack;
  private final Spark[] blinkins;

  public LightsSubsystem(BooleanSupplier TeleOp) {
    isTeleOp = TeleOp;
    blinkinFront = new Spark(LightsConstants.lightPortFront);
    blinkinBack = new Spark(LightsConstants.lightPortBack);
    blinkins = new Spark[] {blinkinFront, blinkinBack};
    reset();
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

  public void signalSource() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.STROBE_GOLD);
  }

  public void setBlue() {
    setColor(blinkins, LightsConstants.Color.BLUE);
  }

  public void setRed() {
    setColor(blinkins, LightsConstants.Color.RED);
  }

  public void setRedAutoAnimation() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.HEARTBEAT_RED);
  }

  public void setBlueAutoAnimation() {
    setPattern(blinkins, LightsConstants.BlinkinPattern.HEARTBEAT_BLUE);
  }

  public void signalNoteInHolder() {
    setColor(blinkins, LightsConstants.Color.GREEN);
  }

  public void displayAllianceColorInAuto() {
    if (isBlue.getAsBoolean() == true) {
      setBlueAutoAnimation();
      return;
    }
    setRedAutoAnimation();
  }

  public void displayAllianceColor() {
    if (isBlue.getAsBoolean() == true) {
      setBlue();
      return;
    }
    setRed();
  }

  public void reset() {
    if (isTeleOp.getAsBoolean()) {
      displayAllianceColor();
      return;
    }
    displayAllianceColorInAuto();
  }
}

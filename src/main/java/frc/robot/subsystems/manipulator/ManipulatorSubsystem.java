package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.DriveConstants;

public class ManipulatorSubsystem implements Subsystem {
  private final IntakeSubsystem intakeSubsystem;

  private ManipulatorState state = ManipulatorState.EMPTY;

  public ManipulatorSubsystem() {
    intakeSubsystem = getIntakeSubsystem();
    setupShuffleboardTab();
  }

  private IntakeSubsystem getIntakeSubsystem() {
    return new IntakeSubsystem();
  }

  private void setupShuffleboardTab() {
    DriveConstants.MANIPULATOR_SYSTEM_TAB.addString("State", () -> state.toString());
  }

  @Override
  public void periodic() {
    // TODO: Implement logic
    switch (state) {
      case PICKUP:
        // Do pickup
        state = ManipulatorState.TRANSITION;
      case TRANSITION:
        // Transition note for pickup
        state = ManipulatorState.SHOOT;
      case SHOOT:
        // Shoot note
        state = ManipulatorState.EMPTY;
      default:
        break;
    }
  }

  public enum ManipulatorState {
    EMPTY("Empty"),
    PICKUP("Pickup"),
    TRANSITION("Transition"),
    SHOOT("Shoot");

    private final String stateDescription;

    ManipulatorState(String stateDescription) {
      this.stateDescription = stateDescription;
    }

    @Override
    public String toString() {
      return stateDescription;
    }
  }
}

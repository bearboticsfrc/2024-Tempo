package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intakeSubsystem;

  private ManipulatorState state = ManipulatorState.EMPTY;

  public ManipulatorSubsystem() {
    intakeSubsystem = new IntakeSubsystem();
    setupShuffleboardTab();
  }

  private void setupShuffleboardTab() {
    DriveConstants.MANIPULATOR_SYSTEM_TAB.addString("State", () -> getState().toString());
  }

  public ManipulatorState getState() {
    return state;
  }

  @Override
  public void periodic() {
    switch (state) {
      case PICKUP:
        // Pickup note
        intakeSubsystem.set(1);
        // state = ManipulatorState.TRANSITION;
        break;
      case TRANSITION:
        // Transition note from pickup
        intakeSubsystem.set(0);
        state = ManipulatorState.SHOOT;
        break;
      case SHOOT:
        // Shoot note
        state = ManipulatorState.EMPTY;
        break;
      default:
        intakeSubsystem.set(0);
        break;
    }
  }

  public void dispatchState(ManipulatorState state) {
    this.state = state;
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

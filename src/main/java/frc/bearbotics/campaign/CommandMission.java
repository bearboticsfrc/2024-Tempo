package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Adapter class for a Command to a Mission */
public class CommandMission extends AbstractMission {
  private final Command command;
  private BooleanSupplier successSupplier = () -> true;

  /**
   * New CommandMission adapter.
   *
   * @param command The command to adapt.
   */
  public CommandMission(final Command command) {
    this.command = command;
    m_requirements.addAll(command.getRequirements());
  }

  /**
   * @see edu.wpi.first.wpilibj2.command.Command#initialize()
   */
  public void initialize() {
    command.initialize();
  }

  /**
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  public void execute() {
    command.execute();
  }

  /**
   * @param interrupted
   * @see edu.wpi.first.wpilibj2.command.Command#end(boolean)
   */
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  /**
   * @return
   * @see edu.wpi.first.wpilibj2.command.Command#isFinished()
   */
  public boolean isFinished() {
    return command.isFinished();
  }

  /**
   * @see edu.wpi.first.wpilibj2.command.Command#schedule()
   */
  public void schedule() {
    command.schedule();
  }

  /**
   * @see edu.wpi.first.wpilibj2.command.Command#cancel()
   */
  public void cancel() {
    command.cancel();
  }

  /**
   * @return
   * @see edu.wpi.first.wpilibj2.command.Command#isScheduled()
   */
  public boolean isScheduled() {
    return command.isScheduled();
  }

  /**
   * @return
   * @see edu.wpi.first.wpilibj2.command.Command#getName()
   */
  public String getName() {
    return command.getName();
  }

  public CommandMission setSuccessCallback(BooleanSupplier callback) {
    this.successSupplier = callback;
    return this;
  }

  public boolean isSuccess() {
    return successSupplier.getAsBoolean();
  }
}

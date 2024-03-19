package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/**
 * {@code CommandMission} serves as an adapter, allowing WPILib {@link Command} objects to be
 * integrated within the mission framework. It wraps a {@code Command} and adapts its lifecycle
 * methods to the {@code AbstractMission} interface.
 */
public class CommandMission extends AbstractMission {
  private final Command command;

  private BooleanSupplier preconditionSupplier = () -> true;
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

  /**
   * Specifies a success condition for the command. The provided callback is evaluated to determine
   * if the command has successfully completed.
   *
   * @param callback A {@link BooleanSupplier} representing the success condition.
   * @return This {@code CommandMission} instance, to allow for method chaining.
   */
  public CommandMission withSuccessCallback(BooleanSupplier callback) {
    this.successSupplier = callback;
    return this;
  }

  /**
   * Evaluates the success condition of the command.
   *
   * @return {@code true} if the success condition is met, {@code false} otherwise.
   */
  public boolean isSuccess() {
    return successSupplier.getAsBoolean();
  }

  /**
   * Specifies a precondition for executing the command. The provided callback is evaluated before
   * the command's execution to determine if the conditions are met for it to proceed.
   *
   * @param callback A {@link BooleanSupplier} representing the precondition.
   * @return This {@code CommandMission} instance, to allow for method chaining.
   */
  public CommandMission withPrecondition(BooleanSupplier callback) {
    this.preconditionSupplier = callback;
    return this;
  }

  /**
   * Evaluates the precondition for executing the command.
   *
   * @return {@code true} if the precondition is met, {@code false} otherwise.
   */
  public boolean isValid() {
    return preconditionSupplier.getAsBoolean();
  }
}

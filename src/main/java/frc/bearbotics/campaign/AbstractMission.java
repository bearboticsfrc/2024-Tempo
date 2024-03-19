package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class AbstractMission extends Command {
  public abstract boolean isSuccess();

  public abstract boolean isValid();
}

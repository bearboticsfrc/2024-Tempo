package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;

public class CampaignExecutor extends Command {
  private Campaign campaign;
  private MissionTree nextMission = null;
  private AbstractMission currentMission = null;

  public CampaignExecutor(Campaign campaign) {
    this.campaign = campaign;
  }

  @Override
  public void initialize() {
    nextMission = campaign.getMissions();
    currentMission = nextMission.getNode();
    currentMission.initialize();
  }

  @Override
  public void execute() {
    currentMission.execute();

    if (!currentMission.isFinished() || nextMission == null) {
      return;
    } else {
      currentMission.end(false);
    }

    nextMission =
        currentMission.isSuccess() ? nextMission.getSuccessNode() : nextMission.getFailureNode();

    if (nextMission == null) {
      currentMission = null; // maybe use this.end()?
      return; // TODO: Logic needs reworking here, maybe
    }

    currentMission = nextMission.getNode();
    currentMission.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    if (currentMission != null) {
      currentMission.end(false);
    }
  }

  @Override
  public boolean isFinished() {
    return currentMission == null;
  }
}

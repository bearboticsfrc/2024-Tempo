package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;

public class CampaignExecutor extends Command {
  private Campaign campaign;
  private MissionTree nextMissionTree = null;
  private AbstractMission currentMission = null;

  public CampaignExecutor(Campaign campaign) {
    this.campaign = campaign;
  }

  @Override
  public void initialize() {
    nextMissionTree = campaign.getMissions();
    currentMission = nextMissionTree.getNode();
    currentMission.initialize();
  }

  @Override
  public void execute() {
    // TODO: Smelly. Needs work.

    if (!currentMission.isValid() && nextMissionTree != null) {

      nextMissionTree = getNextMissionTree(false);
      currentMission = nextMissionTree.getNode();
      currentMission.initialize();
      return;
    }

    currentMission.execute();

    if (!currentMission.isFinished() || nextMissionTree == null) {
      return;
    }

    currentMission.end(false);
    nextMissionTree = getNextMissionTree(currentMission.isSuccess());

    if (nextMissionTree == null) {
      currentMission = null; // maybe use this.end()?
      return;
    }

    currentMission = nextMissionTree.getNode();
    currentMission.initialize();
  }

  private MissionTree getNextMissionTree(boolean previousSuccess) {
    return previousSuccess ? nextMissionTree.getSuccessNode() : nextMissionTree.getFailureNode();
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

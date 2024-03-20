package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj.DataLogManager;
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

    DataLogManager.log("[initalize() - " + currentMission.getName() + "]: Initaizing mission.");
  }

  @Override
  public void execute() {
    DataLogManager.log("[execute() - " + currentMission.getName() + "]: Executing mission.");

    if (!currentMission.isValid() && nextMissionTree != null) {
      DataLogManager.log(
          "[execute() - "
              + currentMission.getName()
              + "]: Precondition did not pass. Getting failure node.");

      nextMissionTree = getNextMissionTree(false);
      currentMission = nextMissionTree.getNode();
      currentMission.initialize();

      DataLogManager.log(
          "[execute() - "
              + currentMission.getName()
              + "]: Failure mission: "
              + nextMissionTree.getNode().getName());
      return;
    }

    currentMission.execute();

    if (!currentMission.isFinished() || nextMissionTree == null) {
      return;
    }

    DataLogManager.log(
        "[execute() - " + currentMission.getName() + "]: Finished mission. Getting next node.");

    currentMission.end(false);
    nextMissionTree = getNextMissionTree(currentMission.isSuccess());

    if (nextMissionTree == null) {
      currentMission = null; // maybe use this.end()?
      return; // TODO: Logic needs reworking here, maybe
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

package frc.bearbotics.campaign;

import edu.wpi.first.wpilibj2.command.Command;

/* A binary tree containing a success node and a failure node. */
public class MissionTree {
  private AbstractMission node;
  private MissionTree successNode = null;
  private MissionTree failureNode = null;

  public MissionTree(AbstractMission node) {
    this.node = node;
  }

  public MissionTree(Command node) {
    this.node = new CommandMission(node);
  }

  /**
   * @return the successNode
   */
  public MissionTree getSuccessNode() {
    return successNode;
  }

  /**
   * @param successNode the successNode to set
   */
  public MissionTree withSuccessNode(MissionTree successNode) {
    this.successNode = successNode;
    return this;
  }

  /**
   * @param successNode the successNode to set
   */
  public MissionTree withSuccessNode(CommandMission successNode) {
    this.successNode = new MissionTree(successNode);
    return this;
  }

  /**
   * @return the failureNode
   */
  public MissionTree getFailureNode() {
    return failureNode;
  }

  /**
   * @param failureNode the failureNode to set
   */
  public MissionTree withFailureNode(MissionTree failureNode) {
    this.failureNode = failureNode;
    return this;
  }

  /**
   * @param failureNode the failureNode to set
   */
  public MissionTree withFailureNode(CommandMission failureNode) {
    this.failureNode = new MissionTree(failureNode);
    return this;
  }

  /**
   * @return the node
   */
  public AbstractMission getNode() {
    return node;
  }

  /**
   * @param node the node to set
   */
  public void setNode(AbstractMission node) {
    this.node = node;
  }
}

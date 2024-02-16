package frc.bearbotics.campaign;

/* A binary tree containing a success node and a failure node. */
public class MissionTree {
  private AbstractMission node;
  private MissionTree successNode = null;
  private MissionTree failureNode = null;

  public MissionTree(AbstractMission node) {
    this.node = node;
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
  public MissionTree setSuccessNode(MissionTree successNode) {
    this.successNode = successNode;
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
  public MissionTree setFailureNode(MissionTree failureNode) {
    this.failureNode = failureNode;
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

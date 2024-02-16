package frc.bearbotics.campaign;

public class Campaign {
  private MissionTree missions;
  private String name;

  public Campaign(String name, MissionTree missions) {
    this.name = name;
    this.missions = missions;
  }

  public Campaign setMissionTree(MissionTree missions) {
    this.missions = missions;
    return this;
  }

  public String getName() {
    return name;
  }

  public MissionTree getMissions() {
    return missions;
  }
}

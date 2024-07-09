package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTag;
import frc.robot.subsystems.vision.VisionCamera;
import java.util.*;
// tag calb will use two average valued camera to tag transform vectors from two different cameras
// on a single tag
// these two cameras will be positioned on two ends of a single side of the robot prefferable aiming
// towards each other(inverted)

// the angle of intersection should ideally be perpendicular
// if the two cameras cannot be positioned towards each other in this manner due to the priority of
// some other robot design constraint
// then such cameras should ideally have their focus aiming towards the tag
public class TagCalbDuo {
  private double kickerZOne;
  private double kickerXOne;

  private double kickerZTwo;
  private double kickerXTwo;

  Vector<Double> idealizedVectorOne;
  Vector<Double> idealizedVectorTwo;

  public TagCalbDuo(
      boolean inverted,
      double knownRobotDistance,
      AprilTag tagToCalb,
      VisionCamera localizingCamera1,
      VisionCamera localizingCamera2) {}

  public Vector<Double> CalibratedVector() {
    return new Vector<Double>();
  }
}

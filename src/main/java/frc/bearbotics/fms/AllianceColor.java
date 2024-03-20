package frc.bearbotics.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AllianceColor {
  public static Alliance alliance = null;
  public static List<AllianceReadyListener> listeners = new ArrayList<AllianceReadyListener>();

  public static void addListener(AllianceReadyListener listener) {
    listeners.add(listener);
  }

  public static void setAllianceColor(Alliance newAlliance) {
    if (alliance == null || alliance != newAlliance) {
      alliance = newAlliance;
      notifyListeners();
    }
  }

  public static Optional<Alliance> getAlliance() {
    return Optional.ofNullable(alliance);
  }

  public static void notifyListeners() {
    for (AllianceReadyListener listener : listeners) {
      listener.updateAllianceColor(alliance);
    }
  }

  public static boolean isRedAlliance() {
    return alliance == Alliance.Red;
  }
}

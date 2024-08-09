package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.localization.CalibratedCamera;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class StoreCalibratedCameras implements Serializable {
  private static final String location = "src/main/deploy/calibrationinfo.obj";
  private final SimpleDateFormat BUILD_DATE_FORMATTER = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
  private String lastCalibration;
  private static final String filename = "calibrationinfo.obj";
  private Date calibrationDate;

  private List<CalibratedCamera> calibratedDuoCameras;

  public static StoreCalibratedCameras fromCalibrateCameras(
      Date buildDate, List<CalibratedCamera> calibratedDuoCameras) {
    StoreCalibratedCameras calibration = new StoreCalibratedCameras();
    calibration.calibratedDuoCameras = calibratedDuoCameras;
    calibration.calibrationDate = buildDate;
    writeObject(calibration);
    return calibration;
  }

  /**
   * get CalibratedDuoCameras
   *
   * @return The CalibratedDuoCameras list
   */
  public List<CalibratedCamera> getCalilCalibratedDuoCameras() {
    return calibratedDuoCameras;
  }

  /**
   * Get the latest build date.
   *
   * @return The build date.
   */
  public Date getCalibrationDate() {
    return calibrationDate;
  }

  /**
   * Publish the stored calibrated cameras info to network tables
   *
   * @param tableName The table name to publish to.
   */
  public void publishCalibrations(String tableName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

    if (table == null) {
      DriverStation.reportError("Could not find NT Table \"" + tableName + "\"", null);
      return;
    }

    table.getEntry("Calibration Date").setString(BUILD_DATE_FORMATTER.format(calibrationDate));
    table.getEntry("Current Calibration").setString(lastCalibration);
  }

  /**
   * Load the calibrated cameras version info from file.
   *
   * @return The StoreCalibratedCameras wrapper object.
   */
  private static StoreCalibratedCameras loadVersion() {
    String path = Filesystem.getDeployDirectory() + "/" + filename;
    StoreCalibratedCameras calibration;

    try {
      FileInputStream fileInputStream = new FileInputStream(path);
      ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
      calibration = (StoreCalibratedCameras) objectInputStream.readObject();
      objectInputStream.close();
    } catch (Exception ignored) {
      return null;
    }

    return calibration;
  }

  public static List<CalibratedCamera> loadStaticVersion(){
    if(StoreCalibratedCameras.loadVersion()==null){
      return null;
    }
    else{
    return StoreCalibratedCameras.loadVersion().getCalilCalibratedDuoCameras();
    }
  }




  /**
   * Execute a shell command using the supplied runtime.
   *
   * @param command The command to run.
   * @param runtime The runtime to run in.
   * @return The command result.
   */

  /**
   * Write a generic object to a file
   *
   * @param location The location to write to.
   * @param object The object to write.
   */
  private static void writeObject(Object object) {
    try {
      FileOutputStream file = new FileOutputStream(location); //
      ObjectOutputStream objectStream = new ObjectOutputStream(file);

      objectStream.writeObject(object);
      objectStream.close();
    } catch (IOException exc) {
      System.exit(1);
      return;
    }
  }
}

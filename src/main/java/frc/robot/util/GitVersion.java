package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.text.SimpleDateFormat;
import java.util.Date;

/** GitVersion class borrowed from team 2832, but cleaned up */
public class GitVersion implements Serializable {
  private final SimpleDateFormat BUILD_DATE_FORMATTER = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");

  private String lastCommit;
  private boolean isModified;
  private Date buildDate;
  private String buildAuthor;

  public static GitVersion fromMetadata(
      String lastCommit, boolean isModified, Date buildDate, String buildAuthor) {
    GitVersion version = new GitVersion();

    version.lastCommit = lastCommit;
    version.isModified = isModified;
    version.buildDate = buildDate;
    version.buildAuthor = buildAuthor;

    return version;
  }

  /**
   * Get latest commit hash.
   * 
   * @return The hash.
   */
  public String getLastCommit() {
    return lastCommit;
  }

  /**
   * Whether the source was modified from the most recent local release.
   * 
   * @return Whether this source was modified.
   */
  public boolean isModified() {
    return isModified;
  }

  /**
   * Get the latest build date.
   * 
   * @return The build date.
   */
  public Date getBuildDate() {
    return buildDate;
  }

  /**
   * Get the latest build author's name.
   * 
   * @return The build author.
   */
  public String getBuildAuthor() {
    return buildAuthor;
  }

  /**
   * Publish the git version info to network tables
   * 
   * @param tableName The table name to publish to.
   */
  public void publishVersions(String tableName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

    if (table == null) {
      DriverStation.reportError("Could not find NT Table \"" + tableName + "\"", null);
      return;
    }

    table.getEntry("Build Date").setString(BUILD_DATE_FORMATTER.format(buildDate));
    table.getEntry("Build Author").setString(buildAuthor);
    table.getEntry("Current Commit").setString(lastCommit);
    table.getEntry("Modified").setBoolean(isModified);
  }

  /**
   * Load the git version info from file.
   * 
   * @return The GitVersion wrapper object.
   */
  public static GitVersion loadVersion(String filename) {
    String path = Filesystem.getDeployDirectory() + "/" + filename;
    GitVersion version;

    try {
      FileInputStream fileInputStream = new FileInputStream(path);
      ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
      version = (GitVersion) objectInputStream.readObject();
      objectInputStream.close();
    } catch (Exception ignored) {
      version = GitVersion.fromMetadata("Unknown", false, new Date(0), "Unknown");
    }

    return version;
  }

  /**
   * Execute a shell command using the supplied runtime.
   * 
   * @param command The command to run.
   * @param runtime The runtime to run in.
   * @return The command result.
   */
  private static String executeCommand(String command, Runtime runtime) {
    Process process;

    try {
      process = runtime.exec(command);
      process.waitFor();

      return new String(process.getInputStream().readAllBytes());
    } catch (InterruptedException | IOException exc) {
      System.out.println(exc);
      System.exit(1);

      return ""; // Practically will never be reached, compiler doesn't know this though
    }
  }

  /**
   * Write a generic object to a file
   * 
   * @param location The location to write to.
   * @param object The object to write.
   */
  private static void writeObject(String location, Object object) {
    try {
      FileOutputStream file = new FileOutputStream(location); // "src/main/deploy/gitinfo.obj"
      ObjectOutputStream objectStream = new ObjectOutputStream(file);

      objectStream.writeObject(object);
      objectStream.close();
    } catch (IOException exc) {
      System.exit(1);
      return;
    }
  }

  // this main function should only be called from Gradle
  public static void main(String[] args) {
    Runtime runtime = Runtime.getRuntime();
    Date buildDate = new Date();
    String buildAuthor = executeCommand("git config user.name", runtime).replace("\n", "");
    String lastCommit = executeCommand("git rev-parse --short HEAD", runtime).replace("\n", "");
    boolean isModified = executeCommand("git status -s", runtime).length() > 0;

    GitVersion version = GitVersion.fromMetadata(lastCommit, isModified, buildDate, buildAuthor);
    writeObject("src/main/deploy/gitinfo.obj", version);

    System.exit(0);
  }
}

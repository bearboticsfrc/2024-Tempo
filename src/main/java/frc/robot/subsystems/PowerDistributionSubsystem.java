package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A subsytem purely for logging purposes. */
public class PowerDistributionSubsystem extends SubsystemBase {
  private final String LOGGING_ROOT = "PDP";

  private final PowerDistribution powerDistribution;

  private final DoubleLogEntry voltageLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "/voltage");
  private final DoubleLogEntry temperatureLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "/temperature");
  private final DoubleLogEntry powerLogEntry =
      new DoubleLogEntry(DataLogManager.getLog(), LOGGING_ROOT + "/power");

  public PowerDistributionSubsystem() {
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    voltageLogEntry.append(powerDistribution.getVoltage());
    temperatureLogEntry.append(powerDistribution.getTemperature());
    powerLogEntry.append(powerDistribution.getTotalPower());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Convenience methods for using the RevRobotics api. */
public class RevUtil {
  private static final int MAXIMUM_ATTEMPS = 5;

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   */
  public static REVLibError checkRevError(REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(error.toString(), true);
    }

    return error;
  }

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   * @param message A message to log with the error.
   */
  public static REVLibError checkRevError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }

    return error;
  }

  public static void set(Supplier<REVLibError> config, String message) {
    for (int attempt = 0; attempt < MAXIMUM_ATTEMPS; attempt++) {
      if (checkRevError(config.get(), message) == REVLibError.kOk) {
        return;
      }
      Timer.delay(0.25);
    }
  }

  /**
   * Sets the Status 0,1,2,3 frame rates for a CANSparkMax motor controller
   *
   * @param motor A CANSparkMax motor controller
   * @param desc Description of the motor ("Climber1", "Left Drive", etc)
   */
  public static void setPeriodicFramePeriodHigh(CANSparkBase motor, String desc) {
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50));
  }
}

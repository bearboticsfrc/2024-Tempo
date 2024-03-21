// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provides convenience methods for handling errors when using the CTRE API. This utility class is
 * designed to simplify the process of error checking when interacting with CTRE devices.
 */
public class CTREUtil {

  /**
   * Checks a CTRE API call's status code for errors. If an error is detected, it reports the error
   * to the DriverStation.
   *
   * @param statusCode The status code returned by a CTRE API call.
   * @return The original status code passed into the method.
   */
  public static StatusCode checkCtreError(StatusCode statusCode) {
    if (statusCode.isError()) {
      DriverStation.reportError(statusCode.toString(), true);
    }

    return statusCode;
  }
}

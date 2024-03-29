// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

/** Convenience methods for using the CTRE api. */
public class CTREUtil {

  /**
   * @param statusCode
   */
  public static StatusCode checkCtreError(StatusCode statusCode) {
    if (statusCode.isError()) {
      DriverStation.reportError(statusCode.toString(), true);
    }

    return statusCode;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

/** Convenience methods for using the CTRE api. */
public class CTREUtil {

  /**
   * @param errorCode
   */
  public static void checkCtreError(ErrorCode errorCode) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(errorCode.toString(), true);
    }
  }

  /**
   * @param errorCode
   * @param message
   */
  public static void checkCtreError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
    }
  }
}

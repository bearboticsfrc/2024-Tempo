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
   * Checks if the result of a REV Robotics API call is successful and logs an error message to the
   * Driver Station if it's not.
   *
   * @param error A REVLibError returned by a REV Robotics API call.
   * @return The same REVLibError passed as a parameter, allowing for chaining or further checks.
   */
  public static REVLibError checkRevError(REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(error.toString(), true);
    }

    return error;
  }

  /**
   * Similar to {@link #checkRevError(REVLibError)}, but allows for a custom message to be logged
   * alongside the error.
   *
   * @param error A REVLibError returned by a REV Robotics API call.
   * @param message A custom message to log with the error, providing additional context.
   * @return The same REVLibError passed as a parameter, allowing for chaining or further checks.
   */
  public static REVLibError checkRevError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }

    return error;
  }

  /**
   * Attempts to apply a configuration multiple times until it succeeds or reaches a maximum number
   * of attempts.
   *
   * @param config A Supplier that returns a REVLibError, representing the configuration command.
   * @param message A message to log if the final attempt fails, providing context for the failure.
   */
  public static void set(Supplier<REVLibError> config, String message) {
    for (int attempt = 0; attempt < MAXIMUM_ATTEMPS - 1; attempt++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }

      Timer.delay(0.25);
    }

    checkRevError(config.get(), message);
  }

  /**
   * Sets the periodic frame periods for the CANSparkMax motor controller to higher rates,
   * optimizing the controller's communication for scenarios requiring high update rates. This
   * method configures the Status 0, 1, 2, and 3 frame rates.
   *
   * @param motor A CANSparkMax motor controller to configure.
   * @param desc A descriptive name for the motor (e.g., "Climber1", "Left Drive"), used for
   *     logging.
   */
  public static void setPeriodicFramePeriodHigh(CANSparkBase motor, String desc) {
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10), desc);
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20), desc);
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20), desc);
    checkRevError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50), desc);
  }
}

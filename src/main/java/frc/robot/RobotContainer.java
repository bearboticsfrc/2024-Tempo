// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.candle.CandleSubsystem;

/**
 * The RobotContainer class serves as the central hub for the robot's system configurations and
 * operations. It initializes all robot subsystems, configures command bindings for both the driver
 * and operator controllers, and sets up autonomous command choices. This class also handles the
 * integration with the Shuffleboard for real-time data display and control adjustments.
 */
public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  @SuppressWarnings("unused")
  private final PowerDistributionSubsystem powerDistributionSubsystem =
      new PowerDistributionSubsystem();

  private final CandleSubsystem candleSubsystem = new CandleSubsystem();

  private boolean isTeleop;
  private boolean isAutoPathTargeting = false;

  public RobotContainer() {
    configureDriverBindings();
  }

  /**
   * Sets up the path planner for autonomous operation. This includes configuring the holonomic path
   * following with appropriate PID constants, setting up replanning configurations, and
   * establishing any global overrides.
   */

  /**
   * Builds and configures the list of autonomous commands available for selection. This method
   * populates the SendableChooser with pre-defined autonomous routines.
   *
   * <p>/** Sets up a list of test commands for debugging and calibration purposes. These commands
   * are accessible from the Test tab on the Shuffleboard.
   */
  private void configureDriverBindings() {}

  /**
   * Configures the button bindings for the driver's Xbox controller. This method maps controller
   * inputs to robot commands for driving, manipulation, and other teleoperated actions.
   */

  /**
   * Retrieves joystick input from a specified axis, applies deadband and scaling, and optionally
   * flips the direction based on alliance color. This method helps with processing raw joystick
   * inputs for driving commands.
   *
   * @param controller The CommandXboxController from which to read the input.
   * @param axis The axis (e.g., Ly, Lx) to read from the controller.
   * @return Processed input value from the specified joystick axis.
   */
  private double getJoystickInput(CommandXboxController controller, JoystickAxis axis) {
    double rawInput;

    switch (axis) {
      case Ly:
        rawInput = driverController.getLeftY();
        break;
      case Lx:
        rawInput = driverController.getLeftX();
        break;
      case Ry:
        rawInput = driverController.getRightY();
        break;
      case Rx:
        rawInput = driverController.getRightX();
        break;
      default:
        rawInput = 0;
    }

    double flippedInput = AllianceColor.isRedAlliance() && axis.isFlipped() ? -rawInput : rawInput;
    return -MathUtil.applyDeadband(powWithSign(flippedInput, 2), 0.01);
  }

  /**
   * Raises a value to a power while preserving the sign, useful for non-linear joystick response
   * curves.
   *
   * @param x The base value.
   * @param b The exponent.
   * @return The result of raising `x` to the power `b`, maintaining the original sign of `x`.
   */
  private double powWithSign(double x, double b) {
    return Math.copySign(Math.pow(x, b), x);
  }

  /**
   * Configures button bindings for the operator's Xbox controller. Similar to driver bindings, this
   * method maps operator inputs to commands for robot manipulation and other functions.
   */

  /**
   * Sets the robot's operational mode to teleoperated and optionally resets odometry.
   *
   * @param mode If true, the robot is set to teleop mode and odometry is reset.
   */
  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  /**
   * Performs initialization tasks when the robot is first started. This includes setting initial
   * subsystem states and configuring global settings.
   */
  public void robotInit() {
    candleSubsystem.setColor(Color.kGreen);
  }

  /** Prepares the robot for being disabled, including stopping any rumble on the controllers. */
  public void disabledInit() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
  /**
   * Enables or disables the auto path targeting mode, which affects autonomous path following
   * behavior.
   *
   * @param isAutoPathTargeting If true, enables auto path targeting mode.
   */
  public void setAutoPathTargeting(boolean isAutoPathTargeting) {
    this.isAutoPathTargeting = isAutoPathTargeting;
  }
  /** Enum defining joystick axes for clearer code when handling joystick inputs. */
  private enum JoystickAxis {
    Ly,
    Lx,
    Ry,
    Rx;

    /**
     * Determines if the axis input should be flipped based on the robot's alliance color.
     *
     * @return true if the axis input is inverted, false otherwise.
     */
    public boolean isFlipped() {
      return this == Ly || this == Lx;
    }
  }
}

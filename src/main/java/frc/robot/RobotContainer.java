// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.candle.CandleSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

/**
 * The RobotContainer class serves as the central hub for the robot's system configurations and
 * operations. It initializes all robot subsystems, configures command bindings for both the driver
 * and operator controllers, and sets up autonomous command choices. This class also handles the
 * integration with the Shuffleboard for real-time data display and control adjustments.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
  private final CandleSubsystem candleSubsystem = new CandleSubsystem();

  public RobotContainer() {
    configureDriverBindings();
  }

  public void robotInit() {
    candleSubsystem.setColor(Color.kBlue);
  }

  /** Sets up a list of test commands for debugging and calibration purposes. */
  private void configureDriverBindings() {
    driverController
        .a()
        .whileTrue(manipulatorSubsystem.getBloopShootCommand())
        .onFalse(manipulatorSubsystem.getShooterStopCommand());
  }
}

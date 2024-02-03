// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private boolean isTeleop;

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    buildAutoList();
    buildTestList();
    configureBindings();
  }

  private RunCommand getDefaultCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
        driveSubsystem);
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());
  }

  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());

    DriveConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  private void buildTestList() {
    DriveConstants.TEST_TAB
        .add(
            "Drive Subsystem Test", new DriveSubsystemTest(driveSubsystem, DriveConstants.TEST_TAB))
        .withPosition(2, 1)
        .withSize(2, 1);
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }
}

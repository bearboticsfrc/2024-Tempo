// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private boolean isTeleop;

  private DriveSubsystem driveSubsystem = new DriveSubsystem();

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    buildAutoList();
    buildTestList();
  }

  private void configureBindings() {}

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
            "Drive Subsystem Test",
            new DriveSubsystemTest(driveSubsystem, DriveConstants.TEST_TAB).withTimeout(5000))
        .withPosition(2, 1)
        .withSize(2, 1);
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }
}

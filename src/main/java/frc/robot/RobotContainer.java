// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.LightsSubsystem;

public class RobotContainer {
  private boolean isTeleop;
  private LightsSubsystem lights = new LightsSubsystem();

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    buildAutoList();
  }

  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private void configureBindings() {
    operatorController.a().onTrue(new InstantCommand(() -> lights.setColor("green")));
    operatorController.b().onTrue(new InstantCommand(() -> lights.setStrip("all")));
    operatorController.x().onTrue(new InstantCommand(() -> lights.setColor("red")));
  }

  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  public void teleopInit() {}

  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());

    DriveConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }
}

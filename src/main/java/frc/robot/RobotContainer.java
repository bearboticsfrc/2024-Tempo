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
import frc.robot.commands.AlignAmpCommand;
import frc.robot.commands.AlignSpeakerCommand;
import frc.robot.commands.notehuntcommand.PointAtNoteCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private boolean isTeleop = false;

  private VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

  public RunCommand getDefaultCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3), 0.1),
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3), 0.1),
                -MathUtil.applyDeadband(Math.pow(driverController.getRightX(), 3), 0.1)),
        driveSubsystem);
  }

  public void teleopInit() {

    driveSubsystem.setSpeedMode(SpeedMode.NORMAL);
  }

  private void configureBindings() {
    configureDriverController();
  }

  private void configureDriverController() {
    driverController.a().onTrue(new InstantCommand(driveSubsystem::zeroHeading));

    driverController
        .x()
        .whileTrue(new PointAtNoteCommand(driveSubsystem, visionSubsystem))
        .onFalse(new Command() {});

    driverController.y().whileTrue(new AlignAmpCommand(driveSubsystem, visionSubsystem));

    driverController.a().whileTrue(new AlignSpeakerCommand(driveSubsystem, visionSubsystem));

    driverController
        .leftBumper()
        .onTrue(new InstantCommand(() -> driveSubsystem.setFieldRelative(false)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setFieldRelative(true)));

    driverController
        .leftTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.NORMAL))));

    driverController
        .rightTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.NORMAL))));
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

  public void buildTestList() {
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

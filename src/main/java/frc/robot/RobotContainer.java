// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  private boolean isTeleop;

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    buildAutoList();
    buildTestList();
    configureDriverBindings();
    configureOperatorBindings();

    DriveConstants.COMPETITION_TAB.add(
        "Home Climber", manipulatorSubsystem.getClimberHomeCommand());
  }

  private RunCommand getDefaultCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3), 0.1),
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3), 0.1),
                -MathUtil.applyDeadband(Math.pow(driverController.getRightX(), 3), 0.1)),
        driveSubsystem);
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());
    driverController
        .leftBumper()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController
        .rightBumper()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController.b().whileTrue(manipulatorSubsystem.getClimberHomeCommand());

    driverController
        .x()
        .whileTrue(manipulatorSubsystem.getIntakeRunCommand())
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController.povDown().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPosition.HOME));

    driverController
        .povUp()
        .whileTrue(manipulatorSubsystem.getArmRunCommand(ArmPosition.AMP_SHOOT))
        .onFalse(manipulatorSubsystem.getArmStopCommand());

    driverController
        .a()
        .whileTrue(manipulatorSubsystem.getShootCommand(3000))
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    new Trigger(manipulatorSubsystem::isNoteInRoller)
        .onTrue(
            new InstantCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
  }

  private void configureOperatorBindings() {
    manipulatorSubsystem.setDefaultCommand(
        manipulatorSubsystem.getClimberRunCommand(
            () -> -MathUtil.applyDeadband(operatorController.getRightY(), 0.1)));
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

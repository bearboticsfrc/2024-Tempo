// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
  private final ObjectDetectionSubsystem objectDetectionSubsystem =
      new ObjectDetectionSubsystem(VisionConstants.OBJECT_DETECTION_CAMERA);

  private boolean isTeleop;

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    buildAutoList();
    buildTestList();
    configureDriverBindings();
    configureOperatorBindings();

    RobotConstants.TUNING_TAB.addDouble(
        "Distance to Pose",
        () ->
            LocationHelper.getDistanceToPose(
                driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter()));

    RobotConstants.COMPETITION_TAB.add(
        "Home Climber", manipulatorSubsystem.getClimberHomeCommand());
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(getDefaultDriveSubsystemCommand());

    driverController
        .leftStick()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController
        .rightStick()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController
        .leftBumper()
        .whileTrue(manipulatorSubsystem.getSubwooferShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .rightBumper()
        .whileTrue(manipulatorSubsystem.getPodiumShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .leftTrigger()
        .whileTrue(manipulatorSubsystem.getIntakeCommand())
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController
        .rightTrigger()
        .whileTrue(manipulatorSubsystem.getRollerRunCommand(IntakeSpeed.REVERSE))
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController.a().onTrue(new InstantCommand(() -> driveSubsystem.resetImu()));

    new Trigger(() -> manipulatorSubsystem.isNoteInRoller() && isTeleop)
        .onTrue(
            new InstantCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .onFalse(
            new InstantCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  private RunCommand getDefaultDriveSubsystemCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(powWithSign(driverController.getLeftY(), 2), 0.01),
                -MathUtil.applyDeadband(powWithSign(driverController.getLeftX(), 3), 0.01),
                -MathUtil.applyDeadband(powWithSign(driverController.getRightX(), 2), 0.01)),
        driveSubsystem);
  }

  /**
   * Raise the first argument to the power of the second argument, keeping the sign of the first.
   *
   * <p>
   *
   * @param x A double.
   * @param b A double.
   * @return The result of x^b with the sign of x.
   */
  private double powWithSign(double x, double b) {
    return Math.copySign(Math.pow(x, b), x);
  }

  private void configureOperatorBindings() {
    manipulatorSubsystem.setDefaultCommand(
        manipulatorSubsystem.getClimberRunCommand(
            () -> -MathUtil.applyDeadband(operatorController.getRightY(), 0.01)));

    operatorController
        .a()
        .whileTrue(manipulatorSubsystem.getAmpShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());
  }

  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());

    RobotConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  private void buildTestList() {
    RobotConstants.TEST_TAB
        .add(
            "Drive Subsystem Test", new DriveSubsystemTest(driveSubsystem, RobotConstants.TEST_TAB))
        .withPosition(2, 1)
        .withSize(2, 1);
  }

  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }

  public void setTeleop(boolean mode) {
    if (mode) {
      driveSubsystem.resetOdometry(new Pose2d(15.071, 5.51, Rotation2d.fromDegrees(180)));
    }

    isTeleop = mode;
  }

  public void disabledInit() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
}
